#ifndef GPU_NOC_NETWORK_HPP
#define GPU_NOC_NETWORK_HPP

#include "gpu_noc_types.hpp"
#include "gpu_noc_router.hpp"
#include <vector>
#include <memory>
#include <unordered_map>
#include <iostream>
#include <iomanip>
#include <algorithm>

namespace gpu_noc
{

    class Network
    {
    public:
        Network(const NoC_Config &config, NetworkType network_type)
            : config_(config), network_type_(network_type), current_cycle_(0),
              routing_engine_(config)
        {

            routers_.reserve(config_.num_routers);
            for (uint32_t i = 0; i < config_.num_routers; i++)
            {
                routers_.push_back(std::make_unique<Router>(i, config_, network_type_));
            }

            std::cout << "[" << network_type_to_string(network_type_) << "] "
                      << "Created " << config_.num_routers << " routers in "
                      << config_.mesh_width << "x" << config_.mesh_height << " mesh\n";
        }

        void cycle()
        {
            // Stage 1 - Deliver/retry flits (lossless)
            process_flit_arrivals();

            // Stage 2: Process credit returns
            process_credit_returns();

            // Stage 3: Routing Computation
            for (auto &router : routers_)
            {
                router->stage_routing_computation();
            }

            // Stage 4: VC Allocation
            for (auto &router : routers_)
            {
                router->stage_vc_allocation();
            }

            // Stage 5: Switch Allocation
            for (auto &router : routers_)
            {
                router->stage_switch_allocation();
            }

            // Stage 6: Switch Traversal - generates flits and credits
            std::unordered_map<uint32_t, std::vector<Flit>> outgoing_flits;
            std::vector<PendingCredit> outgoing_credits;

            for (auto &router : routers_)
            {
                router->stage_switch_traversal(outgoing_flits, outgoing_credits, current_cycle_);
            }

            // Stage 7: Schedule flits and credits for future arrival
            schedule_outgoing_flits(outgoing_flits);
            schedule_outgoing_credits(outgoing_credits);

            current_cycle_++;
        }
        // FLIT INJECTION

        bool inject_flit(uint32_t src_router_id, Flit flit)
        {
            if (src_router_id >= routers_.size())
            {
                return false;
            }

            flit.network_type = network_type_;
            flit.injection_time = current_cycle_;
            flit.current_time = current_cycle_;

            bool success = routers_[src_router_id]->inject_flit(flit, current_cycle_);

            if (success)
            {
                stats_.total_flits_injected[static_cast<uint8_t>(network_type_)]++;
                if (flit.is_head())
                {
                    stats_.total_packets_injected[static_cast<uint8_t>(network_type_)]++;
                }
            }
            else
            {
                stats_.injection_failures++;
            }

            return success;
        }
        // EJECTION WITH BACKPRESSURE

        std::vector<Flit> collect_ejected_flits()
        {
            std::vector<Flit> all_ejected;

            for (auto &router : routers_)
            {
                // Pop at most one flit per router per cycle
                auto flit_opt = router->pop_ejected_flit();

                if (flit_opt.has_value())
                {
                    const Flit &flit = flit_opt.value();
                    all_ejected.push_back(flit);

                    stats_.total_flits_delivered[static_cast<uint8_t>(network_type_)]++;

                    if (flit.is_tail())
                    {
                        stats_.total_packets_delivered[static_cast<uint8_t>(network_type_)]++;

                        uint64_t latency = current_cycle_ - flit.injection_time;
                        stats_.total_latency[static_cast<uint8_t>(network_type_)] += latency;

                        uint8_t idx = static_cast<uint8_t>(network_type_);
                        if (latency > stats_.max_latency[idx])
                        {
                            stats_.max_latency[idx] = latency;
                        }
                        if (latency < stats_.min_latency[idx])
                        {
                            stats_.min_latency[idx] = latency;
                        }

                        stats_.total_hops[idx] += flit.hop_count;
                        if (flit.hop_count > stats_.max_hops[idx])
                        {
                            stats_.max_hops[idx] = flit.hop_count;
                        }
                    }
                }
            }

            return all_ejected;
        }
        // STATISTICS

        const NoC_Stats &get_stats() const
        {
            return stats_;
        }

        void print_stats() const
        {
            uint8_t idx = static_cast<uint8_t>(network_type_);

            std::cout << "\n========================================\n";
            std::cout << network_type_to_string(network_type_) << " Statistics\n";
            std::cout << "========================================\n";
            std::cout << "Packets Injected:   " << stats_.total_packets_injected[idx] << "\n";
            std::cout << "Packets Delivered:  " << stats_.total_packets_delivered[idx] << "\n";
            std::cout << "Flits Injected:     " << stats_.total_flits_injected[idx] << "\n";
            std::cout << "Flits Delivered:    " << stats_.total_flits_delivered[idx] << "\n";
            std::cout << "Injection Failures: " << stats_.injection_failures << "\n";
            std::cout << "Delivery Retries:   " << stats_.delivery_retries << "\n";

            if (stats_.total_packets_delivered[idx] > 0)
            {
                std::cout << std::fixed << std::setprecision(2);
                std::cout << "Avg Latency:        " << stats_.avg_latency(network_type_) << " cycles\n";
                std::cout << "Max Latency:        " << stats_.max_latency[idx] << " cycles\n";
                std::cout << "Min Latency:        " << stats_.min_latency[idx] << " cycles\n";
                std::cout << "Avg Hops:           " << stats_.avg_hops(network_type_) << "\n";
                std::cout << "Max Hops:           " << stats_.max_hops[idx] << "\n";
                std::cout << "Delivery Rate:      " << (stats_.delivery_rate(network_type_) * 100) << "%\n";
            }
            std::cout << "========================================\n\n";
        }

        uint64_t get_current_cycle() const
        {
            return current_cycle_;
        }

        //  Router accessor for collection
        const std::vector<std::unique_ptr<Router>> &get_routers() const
        {
            return routers_;
        }

    private:
        const NoC_Config &config_;
        NetworkType network_type_;
        uint64_t current_cycle_;
        RoutingEngine routing_engine_;

        std::vector<std::unique_ptr<Router>> routers_;
        NoC_Stats stats_;

        std::vector<PendingFlit> flits_in_flight_;
        std::vector<PendingCredit> credits_in_flight_;

        // LOSSLESS SEMANTICS (Hold and Retry)

        void process_flit_arrivals()
        {
            std::vector<PendingFlit> remaining_flits;

            for (auto &pending : flits_in_flight_)
            {
                // Only attempt delivery if flit has arrived
                if (pending.arrival_cycle <= current_cycle_)
                {
                    // Try to deliver (modifies pending.accepted_by_router)
                    deliver_flit_to_router(pending);

                    // If not accepted, count retry
                    if (!pending.accepted_by_router)
                    {
                        stats_.delivery_retries++;
                    }
                }

                // CRITICAL: Keep flit for retry if not accepted (LOSSLESS)
                if (!pending.accepted_by_router)
                {
                    remaining_flits.push_back(pending);
                }
            }

            flits_in_flight_ = std::move(remaining_flits);
        }

        void process_credit_returns()
        {
            std::vector<PendingCredit> remaining_credits;

            for (auto &pending : credits_in_flight_)
            {
                if (pending.arrival_cycle <= current_cycle_)
                {
                    if (pending.router_id < routers_.size())
                    {
                        routers_[pending.router_id]->return_credits(
                            pending.output_port, pending.vc_id);
                    }
                }
                else
                {
                    remaining_credits.push_back(pending);
                }
            }

            credits_in_flight_ = std::move(remaining_credits);
        }

        // Schedule flits for future arrival

        void schedule_outgoing_flits(const std::unordered_map<uint32_t, std::vector<Flit>> &outgoing_flits)
        {
            for (const auto &[dest_router_id, flits] : outgoing_flits)
            {
                for (const auto &flit : flits)
                {
                    uint64_t arrival_cycle = current_cycle_ + config_.link_latency;

                    if (flit.hop_trace.empty())
                    {
                        std::cerr << "[ERROR] Flit has no hop trace!\n";
                        continue;
                    }

                    const HopTrace &last_hop = flit.hop_trace.back();
                    Direction input_port = opposite_dir(last_hop.output_port);

                    flits_in_flight_.emplace_back(flit, dest_router_id, input_port, arrival_cycle);
                }
            }
        }

        //  Schedule credits with correct destination

        void schedule_outgoing_credits(const std::vector<PendingCredit> &outgoing_credits)
        {
            for (const auto &p_credit : outgoing_credits)
            {
                uint32_t current_router = p_credit.router_id;
                Direction flit_input_port = p_credit.output_port;

                // Find upstream router
                uint32_t upstream_router_id;
                try
                {
                    upstream_router_id = routing_engine_.get_neighbor_router_id(
                        current_router, flit_input_port);
                }
                catch (...)
                {
                    // Edge case: injection from LOCAL
                    continue;
                }

                // CRITICAL FIX #1: Credit lands on upstream router's output port
                // If flit came from WEST, credit goes to upstream's EAST port
                Direction credit_dest_port = opposite_dir(flit_input_port);

                uint64_t credit_arrival = current_cycle_ + config_.credit_latency;

                credits_in_flight_.emplace_back(
                    upstream_router_id,
                    credit_dest_port,
                    p_credit.vc_id,
                    credit_arrival);

                if (config_.enable_credit_trace)
                {
                    std::cout << "[CREDIT_SCHED] R" << current_router
                              << " sent credit to R" << upstream_router_id
                              << " landing on port=" << dir_to_string(credit_dest_port)
                              << " vc=" << p_credit.vc_id
                              << " @cycle=" << credit_arrival << "\n";
                }
            }
        }

        //  LOSSLESS delivery (uses reference to modify pending)

        void deliver_flit_to_router(PendingFlit &pending)
        {
            uint32_t router_id = pending.dest_router_id;
            Flit &flit = pending.flit; // Reference for modification
            Direction input_port = pending.input_port;

            if (router_id >= routers_.size())
            {
                std::cerr << "[ERROR] Invalid dest router " << router_id << "\n";
                stats_.dropped_packets++;
                pending.accepted_by_router = true; // Remove from queue
                return;
            }

            auto &router = routers_[router_id];
            uint32_t vc_id = flit.current_vc;

            // Try to deliver
            bool success = router->receive_flit(flit, input_port, vc_id);

            if (success)
            {
                // LOSSLESS: Flit accepted and in buffer
                pending.accepted_by_router = true;
            }
            else
            {
                // LOSSLESS: Flit remains in pending, retry next cycle
                // This should rarely happen if credit contract is correct
            }
        }
    };

    // DUAL NETWORK SYSTEM WITH MULTI-FLIT PACKETIZATION

    class DualNetwork
    {
    public:
        explicit DualNetwork(const NoC_Config &config)
            : config_(config)
        {

            request_network_ = std::make_unique<Network>(config_, NetworkType::REQUEST);
            response_network_ = std::make_unique<Network>(config_, NetworkType::RESPONSE);

            initialize_endpoint_mapping();

            std::cout << "\nDual Network Initialized:\n";
            std::cout << "  Mesh: " << config_.mesh_width << "x" << config_.mesh_height << "\n";
            std::cout << "  Shader Cores: " << config_.num_shader_cores << "\n";
            std::cout << "  L2 Banks: " << config_.num_l2_banks << "\n";
            std::cout << "  VCs per Port: " << config_.num_vcs_per_port << "\n";
            std::cout << "  VC Buffer Depth: " << config_.vc_buffer_depth << "\n";
            std::cout << "  Link Latency: " << config_.link_latency << " cycles\n";
            std::cout << "  Credit Latency: " << config_.credit_latency << " cycles\n";
            std::cout << "  Max Packet Size: " << config_.max_packet_size_flits << " flits\n";
            std::cout << "  Ejection Queue Depth: 8 flits (per router)\n";
            print_endpoint_mapping();
        }

        // CORRECT ENDPOINT MAPPING

        void initialize_endpoint_mapping()
        {
            // SMs on routers 0, 1, 2, ...
            for (uint32_t sm_id = 0; sm_id < config_.num_shader_cores; sm_id++)
            {
                uint32_t router_id = sm_id % config_.num_routers;
                sm_to_router_[sm_id] = router_id;
                router_to_sms_[router_id].push_back(sm_id);
            }

            // CRITICAL: L2 banks offset to avoid SM collisions
            for (uint32_t l2_id = 0; l2_id < config_.num_l2_banks; l2_id++)
            {
                uint32_t router_id = (config_.num_shader_cores + l2_id) % config_.num_routers;
                l2_to_router_[l2_id] = router_id;
                router_to_l2s_[router_id].push_back(l2_id);
            }
        }

        void print_endpoint_mapping() const
        {
            std::cout << "\n=== Endpoint to Router Mapping ===\n";

            std::cout << "\nShader Cores:\n";
            for (const auto &[sm_id, router_id] : sm_to_router_)
            {
                uint32_t x, y;
                config_.router_id_to_xy(router_id, x, y);
                std::cout << "  SM[" << sm_id << "] -> Router[" << router_id
                          << "] @ (" << x << "," << y << ")\n";
            }

            std::cout << "\nL2 Cache Banks:\n";
            for (const auto &[l2_id, router_id] : l2_to_router_)
            {
                uint32_t x, y;
                config_.router_id_to_xy(router_id, x, y);
                std::cout << "  L2[" << l2_id << "] -> Router[" << router_id
                          << "] @ (" << x << "," << y << ")\n";
            }
            std::cout << "==================================\n\n";
        }

        // MULTI-FLIT PACKET INJECTION

        bool send_read_request(uint32_t sm_id, uint32_t l2_id, uint64_t address,
                               uint64_t packet_id)
        {
            Flit flit = create_request_flit_base(sm_id, l2_id, address, packet_id,
                                                 TrafficType::READ_REQUEST);
            flit.packet_size = 1;
            flit.flit_id = 0;
            flit.flit_type = FlitType::HEAD_TAIL;
            flit.current_vc = 0;

            return request_network_->inject_flit(sm_to_router_[sm_id], flit);
        }

        bool send_write_request(uint32_t sm_id, uint32_t l2_id, uint64_t address,
                                uint64_t packet_id, const std::vector<uint8_t> &data)
        {
            Flit flit = create_request_flit_base(sm_id, l2_id, address, packet_id,
                                                 TrafficType::WRITE_REQUEST);
            flit.packet_size = 1;
            flit.flit_id = 0;
            flit.flit_type = FlitType::HEAD_TAIL;
            flit.data = data;
            flit.current_vc = (config_.num_vcs_per_port > 1) ? 1 : 0;

            return request_network_->inject_flit(sm_to_router_[sm_id], flit);
        }

        // MULTI-FLIT Read Response
        bool send_read_response(uint32_t l2_id, uint32_t sm_id, uint64_t packet_id,
                                const std::vector<uint8_t> &data)
        {
            uint32_t packet_size = config_.max_packet_size_flits;
            uint32_t src_router = l2_to_router_[l2_id];

            bool all_success = true;
            for (uint32_t i = 0; i < packet_size; ++i)
            {
                Flit flit = create_response_flit_base(l2_id, sm_id, packet_id,
                                                      TrafficType::READ_RESPONSE);
                flit.packet_size = packet_size;
                flit.flit_id = i;

                if (packet_size == 1)
                {
                    flit.flit_type = FlitType::HEAD_TAIL;
                }
                else if (i == 0)
                {
                    flit.flit_type = FlitType::HEAD;
                }
                else if (i == packet_size - 1)
                {
                    flit.flit_type = FlitType::TAIL;
                }
                else
                {
                    flit.flit_type = FlitType::BODY;
                }

                if (!data.empty())
                {
                    size_t bytes_per_flit = config_.flit_size_bytes;
                    size_t start_byte = i * bytes_per_flit;
                    size_t end_byte = std::min(start_byte + bytes_per_flit, data.size());

                    if (start_byte < data.size())
                    {
                        flit.data.assign(data.begin() + start_byte,
                                         data.begin() + end_byte);
                    }
                }

                flit.current_vc = 0;

                if (!response_network_->inject_flit(src_router, flit))
                {
                    all_success = false;
                    break;
                }
            }

            return all_success;
        }

        bool send_write_response(uint32_t l2_id, uint32_t sm_id, uint64_t packet_id)
        {
            Flit flit = create_response_flit_base(l2_id, sm_id, packet_id,
                                                  TrafficType::WRITE_RESPONSE);
            flit.packet_size = 1;
            flit.flit_id = 0;
            flit.flit_type = FlitType::HEAD_TAIL;
            flit.current_vc = (config_.num_vcs_per_port > 1) ? 1 : 0;

            return response_network_->inject_flit(l2_to_router_[l2_id], flit);
        }

        void cycle()
        {
            request_network_->cycle();
            response_network_->cycle();
        }

        void run_cycles(uint64_t num_cycles)
        {
            for (uint64_t i = 0; i < num_cycles; i++)
            {
                cycle();
            }
        }

        std::vector<Flit> collect_sm_arrivals()
        {
            return response_network_->collect_ejected_flits();
        }

        std::vector<Flit> collect_l2_arrivals()
        {
            return request_network_->collect_ejected_flits();
        }

        void print_statistics() const
        {
            request_network_->print_stats();
            response_network_->print_stats();

            auto req_stats = request_network_->get_stats();
            auto resp_stats = response_network_->get_stats();

            std::cout << "\n========================================\n";
            std::cout << "Combined Network Statistics\n";
            std::cout << "========================================\n";
            std::cout << "Total Packets Delivered: "
                      << (req_stats.total_packets_delivered[0] + resp_stats.total_packets_delivered[1])
                      << "\n";
            std::cout << "Total Flits Delivered:   "
                      << (req_stats.total_flits_delivered[0] + resp_stats.total_flits_delivered[1])
                      << "\n";
            std::cout << "Total Injection Failures: "
                      << (req_stats.injection_failures + resp_stats.injection_failures) << "\n";
            std::cout << "Total Delivery Retries:   "
                      << (req_stats.delivery_retries + resp_stats.delivery_retries) << "\n";
            std::cout << "========================================\n\n";
        }

        const NoC_Stats &get_request_network_stats() const
        {
            return request_network_->get_stats();
        }

        const NoC_Stats &get_response_network_stats() const
        {
            return response_network_->get_stats();
        }

    private:
        const NoC_Config &config_;
        std::unique_ptr<Network> request_network_;
        std::unique_ptr<Network> response_network_;

        std::unordered_map<uint32_t, uint32_t> sm_to_router_;
        std::unordered_map<uint32_t, uint32_t> l2_to_router_;
        std::unordered_map<uint32_t, std::vector<uint32_t>> router_to_sms_;
        std::unordered_map<uint32_t, std::vector<uint32_t>> router_to_l2s_;

        Flit create_request_flit_base(uint32_t sm_id, uint32_t l2_id, uint64_t address,
                                      uint64_t packet_id, TrafficType type)
        {
            Flit flit;
            flit.packet_id = packet_id;
            flit.traffic_type = type;
            flit.network_type = NetworkType::REQUEST;

            flit.src_router_id = sm_to_router_[sm_id];
            flit.dst_router_id = l2_to_router_[l2_id];
            config_.router_id_to_xy(flit.src_router_id, flit.src_x, flit.src_y);
            config_.router_id_to_xy(flit.dst_router_id, flit.dst_x, flit.dst_y);

            flit.src_endpoint_id = sm_id;
            flit.dst_endpoint_id = l2_id;
            flit.src_endpoint_type = EndpointType::SHADER_CORE;
            flit.dst_endpoint_type = EndpointType::L2_BANK;

            flit.address = address;
            flit.hop_count = 0;

            return flit;
        }

        Flit create_response_flit_base(uint32_t l2_id, uint32_t sm_id, uint64_t packet_id,
                                       TrafficType type)
        {
            Flit flit;
            flit.packet_id = packet_id;
            flit.traffic_type = type;
            flit.network_type = NetworkType::RESPONSE;

            flit.src_router_id = l2_to_router_[l2_id];
            flit.dst_router_id = sm_to_router_[sm_id];
            config_.router_id_to_xy(flit.src_router_id, flit.src_x, flit.src_y);
            config_.router_id_to_xy(flit.dst_router_id, flit.dst_x, flit.dst_y);

            flit.src_endpoint_id = l2_id;
            flit.dst_endpoint_id = sm_id;
            flit.src_endpoint_type = EndpointType::L2_BANK;
            flit.dst_endpoint_type = EndpointType::SHADER_CORE;

            flit.hop_count = 0;

            return flit;
        }
    };

} // namespace gpu_noc

#endif // GPU_NOC_NETWORK_HPP