#ifndef GPU_NOC_ROUTER_HPP
#define GPU_NOC_ROUTER_HPP

#include "gpu_noc_types.hpp"
#include "gpu_noc_routing.hpp"
#include "gpu_noc_vc_buffer.hpp"
#include <memory>
#include <unordered_map>
#include <deque>
#include <algorithm>
#include <iostream>

namespace gpu_noc
{

    class Router
    {
    public:
        Router(uint32_t router_id, const NoC_Config &config, NetworkType network_type)
            : router_id_(router_id),
              config_(config),
              network_type_(network_type),
              routing_engine_(config),
              vc_selector_(config.num_vcs_per_port),
              EJECTION_QUEUE_DEPTH_(8)
        {

            config_.router_id_to_xy(router_id_, x_, y_);

            for (uint32_t i = 0; i < static_cast<uint32_t>(Direction::NUM_DIRECTIONS); i++)
            {
                Direction dir = static_cast<Direction>(i);
                input_ports_.emplace_back(dir, config_.num_vcs_per_port, config_.vc_buffer_depth);
            }

            credit_manager_ = std::make_unique<CreditManager>(
                static_cast<uint32_t>(Direction::NUM_DIRECTIONS),
                config_.num_vcs_per_port,
                config_.vc_buffer_depth,
                config_.enable_credit_trace);

            last_grant_.resize(static_cast<uint32_t>(Direction::NUM_DIRECTIONS), 0);

            for (uint32_t i = 0; i < static_cast<uint32_t>(Direction::NUM_DIRECTIONS); i++)
            {
                Direction dir = static_cast<Direction>(i);
                output_vc_busy_[dir].resize(config_.num_vcs_per_port, false);
            }
        }

        uint32_t get_id() const { return router_id_; }
        uint32_t get_x() const { return x_; }
        uint32_t get_y() const { return y_; }
        NetworkType get_network_type() const { return network_type_; }

        // STAGE 1: ROUTING COMPUTATION (RC)

        void stage_routing_computation()
        {
            for (auto &input_port : input_ports_)
            {
                for (uint32_t vc_id = 0; vc_id < input_port.num_vcs(); vc_id++)
                {
                    auto vc_state = input_port.get_vc_state(vc_id);
                    auto &vc_buffer = input_port.get_vc(vc_id);
                    auto flit_opt = vc_buffer.front();

                    // Wormhole: If ACTIVE, skip RC
                    if (vc_state == InputPort::VCState::ACTIVE)
                    {
                        continue;
                    }

                    // Only process HEAD flits
                    if (vc_state == InputPort::VCState::ROUTING ||
                        vc_state == InputPort::VCState::IDLE)
                    {

                        if (flit_opt && flit_opt->is_head())
                        {
                            Direction output_dir = routing_engine_.compute_route(*flit_opt, router_id_);

                            input_port.set_output_port(vc_id, output_dir);

                            if (output_dir == Direction::LOCAL)
                            {
                                input_port.set_vc_state(vc_id, InputPort::VCState::ACTIVE);
                            }
                            else
                            {
                                input_port.set_vc_state(vc_id, InputPort::VCState::VC_ALLOCATED);
                            }

                            if (config_.enable_debug_trace)
                            {
                                std::cout << "[RC] R" << router_id_
                                          << " pkt=" << flit_opt->packet_id
                                          << " flit=" << flit_opt->flit_id
                                          << " type=" << flit_type_to_string(flit_opt->flit_type)
                                          << " in_port=" << dir_to_string(input_port.get_direction())
                                          << " in_vc=" << vc_id
                                          << " → out_port=" << dir_to_string(output_dir) << "\n";
                            }
                        }
                    }
                }
            }
        }

        // STAGE 2: VIRTUAL CHANNEL ALLOCATION (VA)

        void stage_vc_allocation()
        {
            for (auto &input_port : input_ports_)
            {
                for (uint32_t vc_id = 0; vc_id < input_port.num_vcs(); vc_id++)
                {

                    if (input_port.get_vc_state(vc_id) == InputPort::VCState::VC_ALLOCATED)
                    {
                        Direction output_dir = input_port.get_output_port(vc_id);

                        for (uint32_t out_vc = 0; out_vc < config_.num_vcs_per_port; out_vc++)
                        {
                            if (is_output_vc_free(output_dir, out_vc))
                            {

                                //  Check downstream credit before allocation
                                if (!credit_manager_->has_credit(output_dir, out_vc))
                                {
                                    if (config_.enable_debug_trace)
                                    {
                                        std::cout << "[VA] R" << router_id_
                                                  << " stalled: out_port=" << dir_to_string(output_dir)
                                                  << " out_vc=" << out_vc
                                                  << " has ZERO credits downstream\n";
                                    }
                                    continue;
                                }

                                // Allocation successful
                                input_port.set_output_vc(vc_id, out_vc);
                                mark_output_vc_busy(output_dir, out_vc);
                                input_port.set_vc_state(vc_id, InputPort::VCState::ACTIVE);

                                if (config_.enable_debug_trace)
                                {
                                    std::cout << "[VA] R" << router_id_
                                              << " allocated out_vc=" << out_vc
                                              << " for in_port=" << dir_to_string(input_port.get_direction())
                                              << " in_vc=" << vc_id
                                              << " (wormhole locked, credits="
                                              << credit_manager_->get_credits(output_dir, out_vc) << ")\n";
                                }
                                break;
                            }
                        }
                    }
                }
            }
        }

        // STAGE 3: SWITCH ALLOCATION (SA)

        struct SwitchRequest
        {
            Direction input_port;
            uint32_t input_vc;
            Direction output_port;
            uint32_t output_vc;
            Flit flit;
        };

        void stage_switch_allocation()
        {
            switch_grants_.clear();

            std::vector<SwitchRequest> requests;

            for (auto &input_port : input_ports_)
            {
                for (uint32_t vc_id = 0; vc_id < input_port.num_vcs(); vc_id++)
                {

                    if (input_port.get_vc_state(vc_id) == InputPort::VCState::ACTIVE)
                    {
                        auto &vc_buffer = input_port.get_vc(vc_id);

                        if (!vc_buffer.is_empty())
                        {
                            auto flit_opt = vc_buffer.front();
                            if (flit_opt)
                            {
                                Direction out_port = input_port.get_output_port(vc_id);
                                uint32_t out_vc = input_port.get_output_vc(vc_id);

                                // For external links: check credit
                                // For LOCAL: check ejection queue
                                bool can_grant = false;

                                if (out_port == Direction::LOCAL)
                                {
                                    // Check if ejection queue has space for ENTIRE packet
                                    // This prevents partial packet ejection and VC deadlock
                                    uint32_t required_space = flit_opt->packet_size;
                                    uint32_t available_space = EJECTION_QUEUE_DEPTH_ - ejection_queue_.size();

                                    if (available_space >= required_space)
                                    {
                                        can_grant = true;
                                    }
                                    else
                                    {
                                        can_grant = false;
                                        if (config_.enable_debug_trace)
                                        {
                                            std::cout << "[SA] R" << router_id_
                                                      << " blocked: Ejection Queue needs " << required_space
                                                      << " slots, only " << available_space << " available"
                                                      << " (size=" << ejection_queue_.size()
                                                      << "/" << EJECTION_QUEUE_DEPTH_ << ")\n";
                                        }
                                    }
                                }
                                else
                                {
                                    if (credit_manager_->has_credit(out_port, out_vc))
                                    {
                                        can_grant = true;
                                    }
                                    else if (config_.enable_credit_trace)
                                    {
                                        std::cout << "[SA] R" << router_id_
                                                  << " blocked: no credit for out_port="
                                                  << dir_to_string(out_port)
                                                  << " out_vc=" << out_vc << "\n";
                                    }
                                }

                                if (can_grant)
                                {
                                    SwitchRequest req;
                                    req.input_port = input_port.get_direction();
                                    req.input_vc = vc_id;
                                    req.output_port = out_port;
                                    req.output_vc = out_vc;
                                    req.flit = *flit_opt;
                                    requests.push_back(req);
                                }
                            }
                        }
                    }
                }
            }

            // Arbitrate: round-robin per output port
            std::unordered_map<Direction, std::vector<SwitchRequest>> requests_per_output;
            for (const auto &req : requests)
            {
                requests_per_output[req.output_port].push_back(req);
            }

            for (auto &[out_port, port_requests] : requests_per_output)
            {
                if (port_requests.empty())
                    continue;

                uint32_t out_idx = static_cast<uint32_t>(out_port);
                uint32_t start_idx = last_grant_[out_idx];

                for (size_t i = 0; i < port_requests.size(); i++)
                {
                    size_t idx = (start_idx + i) % port_requests.size();
                    const auto &req = port_requests[idx];

                    switch_grants_.push_back(req);
                    last_grant_[out_idx] = (idx + 1) % port_requests.size();

                    if (config_.enable_debug_trace)
                    {
                        std::cout << "[SA] R" << router_id_
                                  << " granted pkt=" << req.flit.packet_id
                                  << " flit=" << req.flit.flit_id
                                  << " type=" << flit_type_to_string(req.flit.flit_type)
                                  << " in_port=" << dir_to_string(req.input_port)
                                  << " in_vc=" << req.input_vc
                                  << " → out_port=" << dir_to_string(req.output_port)
                                  << " out_vc=" << req.output_vc << "\n";
                    }
                    break;
                }
            }
        }

        // STAGE 4: SWITCH TRAVERSAL (ST)

        void stage_switch_traversal(
            std::unordered_map<uint32_t, std::vector<Flit>> &outgoing_flits,
            std::vector<PendingCredit> &credits_to_schedule,
            uint64_t current_cycle)
        {

            for (const auto &grant : switch_grants_)
            {
                uint32_t in_port_idx = static_cast<uint32_t>(grant.input_port);
                auto &input_port = input_ports_[in_port_idx];
                auto &vc_buffer = input_port.get_vc(grant.input_vc);

                // Ensure the flit we are popping is actually there
                auto flit_opt = vc_buffer.pop();
                if (!flit_opt)
                {
                    if (config_.enable_debug_trace)
                    {
                        std::cout << "[ST] R" << router_id_
                                  << " WARNING: Grant without flit in buffer\n";
                    }
                    continue;
                }

                Flit flit = *flit_opt;

                // Generate credit upon POP
                Direction credit_dest_port = grant.input_port;
                credits_to_schedule.emplace_back(
                    router_id_,
                    credit_dest_port,
                    grant.input_vc,
                    current_cycle);

                // Update flit
                flit.current_vc = grant.output_vc;
                flit.allocated_out_vc = grant.output_vc;
                flit.current_time = current_cycle;

                // Record hop
                flit.record_hop(router_id_, grant.input_port, grant.output_port,
                                grant.input_vc, current_cycle);

                //  Consume credit ONLY for external links
                if (grant.output_port != Direction::LOCAL)
                {
                    credit_manager_->consume_credit(grant.output_port, grant.output_vc, router_id_);

                    if (config_.enable_credit_trace)
                    {
                        std::cout << "[ST] R" << router_id_
                                  << " consumed credit: out_port=" << dir_to_string(grant.output_port)
                                  << " out_vc=" << grant.output_vc
                                  << " remaining=" << credit_manager_->get_credits(grant.output_port, grant.output_vc)
                                  << "\n";
                    }
                }
                else
                {
                    if (config_.enable_debug_trace)
                    {
                        std::cout << "[ST] R" << router_id_
                                  << " LOCAL ejection (no credit consumption, uses ejection queue)\n";
                    }
                }

                if (flit.is_tail())
                {
                    mark_output_vc_free(grant.output_port, grant.output_vc);
                    input_port.set_vc_state(grant.input_vc, InputPort::VCState::IDLE);

                    if (config_.enable_debug_trace)
                    {
                        std::cout << "[ST] R" << router_id_
                                  << " TAIL flit freed wormhole: pkt=" << flit.packet_id
                                  << " out_port=" << dir_to_string(grant.output_port)
                                  << " out_vc=" << grant.output_vc << "\n";
                    }
                }

                // Eject or forward
                if (grant.output_port == Direction::LOCAL)
                {
                    handle_ejection(flit);
                }
                else
                {
                    uint32_t next_router_id = routing_engine_.get_neighbor_router_id(
                        router_id_, grant.output_port);

                    if (outgoing_flits.find(next_router_id) == outgoing_flits.end())
                    {
                        outgoing_flits[next_router_id] = std::vector<Flit>();
                    }
                    outgoing_flits[next_router_id].push_back(flit);

                    if (config_.enable_debug_trace)
                    {
                        std::cout << "[ST] R" << router_id_
                                  << " sent " << flit_type_to_string(flit.flit_type)
                                  << " pkt=" << flit.packet_id
                                  << " flit=" << flit.flit_id
                                  << " to R" << next_router_id
                                  << " on VC=" << flit.current_vc << "\n";
                    }
                }
            }

            //  Clear grants to prevent re-processing in next cycle
            switch_grants_.clear();
        }

        // FLIT INJECTION

        bool inject_flit(Flit &flit, uint64_t current_cycle)
        {
            uint32_t local_idx = static_cast<uint32_t>(Direction::LOCAL);
            auto &local_port = input_ports_[local_idx];

            // Use VC assigned by DualNetwork
            uint32_t vc_id = flit.current_vc;

            if (vc_id >= local_port.num_vcs())
            {
                if (config_.enable_debug_trace)
                {
                    std::cout << "[INJECT] R" << router_id_
                              << " ERROR: invalid vc_id=" << vc_id << "\n";
                }
                return false;
            }

            auto &vc_buffer = local_port.get_vc(vc_id);

            if (!vc_buffer.has_space())
            {
                if (config_.enable_debug_trace)
                {
                    std::cout << "[INJECT] R" << router_id_
                              << " FULL: VC=" << vc_id
                              << " pkt=" << flit.packet_id
                              << " flit=" << flit.flit_id << "\n";
                }
                return false;
            }

            flit.injection_time = current_cycle;
            flit.current_time = current_cycle;

            if (!vc_buffer.push(flit))
            {
                return false;
            }

            if (flit.is_head())
            {
                local_port.set_vc_state(vc_id, InputPort::VCState::ROUTING);
            }

            if (config_.enable_debug_trace)
            {
                std::cout << "[INJECT] R" << router_id_
                          << " injected " << flit_type_to_string(flit.flit_type)
                          << " pkt=" << flit.packet_id
                          << " flit=" << flit.flit_id
                          << " into VC=" << vc_id << "\n";
            }

            return true;
        }

        // FLIT RECEPTION

        bool receive_flit(const Flit &flit, Direction input_dir, uint32_t vc_id)
        {
            uint32_t port_idx = static_cast<uint32_t>(input_dir);
            auto &input_port = input_ports_[port_idx];

            if (vc_id >= input_port.num_vcs())
            {
                if (config_.enable_debug_trace)
                {
                    std::cout << "[RECEIVE] R" << router_id_
                              << " ERROR: vc_id=" << vc_id << " >= num_vcs\n";
                }
                return false;
            }

            auto &vc_buffer = input_port.get_vc(vc_id);

            if (!vc_buffer.has_space())
            {
                if (config_.enable_debug_trace)
                {
                    std::cout << "[RECEIVE] R" << router_id_
                              << " RETRY: " << flit_type_to_string(flit.flit_type)
                              << " pkt=" << flit.packet_id
                              << " flit=" << flit.flit_id
                              << " port=" << dir_to_string(input_dir)
                              << " vc=" << vc_id << " (buffer full)\n";
                }
                return false;
            }

            if (!vc_buffer.push(flit))
            {
                return false;
            }

            if (flit.is_head())
            {
                input_port.set_vc_state(vc_id, InputPort::VCState::ROUTING);
            }

            if (config_.enable_debug_trace)
            {
                std::cout << "[RECEIVE] R" << router_id_
                          << " received " << flit_type_to_string(flit.flit_type)
                          << " pkt=" << flit.packet_id
                          << " flit=" << flit.flit_id
                          << " port=" << dir_to_string(input_dir)
                          << " vc=" << vc_id << "\n";
            }

            return true;
        }

        void return_credits(Direction output_port, uint32_t vc_id)
        {
            credit_manager_->return_credit(output_port, vc_id, router_id_);
        }
        // EJECTION QUEUE

        std::optional<Flit> pop_ejected_flit()
        {
            if (ejection_queue_.empty())
            {
                return std::nullopt;
            }

            Flit flit = ejection_queue_.front();
            ejection_queue_.pop_front();
            return flit;
        }

        size_t get_ejection_queue_size() const
        {
            return ejection_queue_.size();
        }

        bool is_ejection_queue_full() const
        {
            return ejection_queue_.size() >= EJECTION_QUEUE_DEPTH_;
        }

    private:
        uint32_t router_id_;
        uint32_t x_, y_;
        const NoC_Config &config_;
        NetworkType network_type_;

        RoutingEngine routing_engine_;
        VCSelector vc_selector_;
        std::vector<InputPort> input_ports_;
        std::unique_ptr<CreditManager> credit_manager_;

        std::unordered_map<Direction, std::vector<bool>> output_vc_busy_;
        std::vector<uint32_t> last_grant_;
        std::vector<SwitchRequest> switch_grants_;

        std::deque<Flit> ejection_queue_;
        const uint32_t EJECTION_QUEUE_DEPTH_;

        bool is_output_vc_free(Direction port, uint32_t vc_id)
        {
            if (output_vc_busy_.find(port) == output_vc_busy_.end())
            {
                output_vc_busy_[port].resize(config_.num_vcs_per_port, false);
            }
            return !output_vc_busy_[port][vc_id];
        }

        void mark_output_vc_busy(Direction port, uint32_t vc_id)
        {
            if (output_vc_busy_.find(port) == output_vc_busy_.end())
            {
                output_vc_busy_[port].resize(config_.num_vcs_per_port, false);
            }
            output_vc_busy_[port][vc_id] = true;
        }

        void mark_output_vc_free(Direction port, uint32_t vc_id)
        {
            if (output_vc_busy_.find(port) != output_vc_busy_.end())
            {
                output_vc_busy_[port][vc_id] = false;
            }
        }

        void handle_ejection(const Flit &flit)
        {
            // SA already verified space
            if (ejection_queue_.size() < EJECTION_QUEUE_DEPTH_)
            {
                ejection_queue_.push_back(flit);

                if (config_.enable_debug_trace)
                {
                    std::cout << "[EJECT] R" << router_id_
                              << " queued " << flit_type_to_string(flit.flit_type)
                              << " pkt=" << flit.packet_id
                              << " flit=" << flit.flit_id
                              << " endpoint=" << flit.dst_endpoint_id
                              << " queue_size=" << ejection_queue_.size() << "\n";
                }
            }
            else
            {
                // Should NEVER happen after SA check
                std::cerr << "[EJECT] R" << router_id_
                          << " CRITICAL ERROR: Ejection queue FULL after SA check!\n"
                          << "  This indicates a logic error in SA stage.\n"
                          << "  pkt=" << flit.packet_id << " flit=" << flit.flit_id << "\n";
            }
        }
    };

}

#endif