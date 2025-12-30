#ifndef GPU_NOC_VC_BUFFER_HPP
#define GPU_NOC_VC_BUFFER_HPP

#include "gpu_noc_types.hpp"
#include <queue>
#include <optional>
#include <deque>
#include <iostream>

namespace gpu_noc
{

    // VIRTUAL CHANNEL BUFFER

    class VCBuffer
    {
    public:
        explicit VCBuffer(uint32_t depth = 4)
            : depth_(depth), credits_(depth) {}

        bool has_space() const { return buffer_.size() < depth_; }
        bool is_empty() const { return buffer_.empty(); }
        bool is_full() const { return buffer_.size() >= depth_; }
        size_t size() const { return buffer_.size(); }
        uint32_t depth() const { return depth_; }
        double occupancy() const
        {
            return depth_ > 0 ? static_cast<double>(buffer_.size()) / depth_ : 0.0;
        }

        bool push(const Flit &flit)
        {
            if (!has_space())
                return false;
            buffer_.push_back(flit);
            return true;
        }

        std::optional<Flit> front() const
        {
            if (buffer_.empty())
                return std::nullopt;
            return buffer_.front();
        }

        std::optional<Flit> pop()
        {
            if (buffer_.empty())
                return std::nullopt;
            Flit flit = buffer_.front();
            buffer_.pop_front();
            return flit;
        }

        std::optional<Flit> peek_at(size_t index) const
        {
            if (index >= buffer_.size())
                return std::nullopt;
            return buffer_[index];
        }

        void clear()
        {
            buffer_.clear();
            reset_credits();
        }

        uint32_t get_credits() const { return credits_; }
        bool has_credit() const { return credits_ > 0; }
        void consume_credit()
        {
            if (credits_ > 0)
                credits_--;
        }
        void return_credit()
        {
            if (credits_ < depth_)
                credits_++;
        }
        void reset_credits() { credits_ = depth_; }
        void set_credits(uint32_t credits)
        {
            credits_ = (credits <= depth_) ? credits : depth_;
        }

    private:
        uint32_t depth_;
        uint32_t credits_;
        std::deque<Flit> buffer_;
    };

    // INPUT PORT

    class InputPort
    {
    public:
        enum class VCState
        {
            IDLE,
            ROUTING,
            VC_ALLOCATED,
            ACTIVE
        };

        InputPort(Direction dir, uint32_t num_vcs, uint32_t vc_depth)
            : direction_(dir), num_vcs_(num_vcs)
        {

            vc_buffers_.reserve(num_vcs);
            for (uint32_t i = 0; i < num_vcs; i++)
            {
                vc_buffers_.emplace_back(vc_depth);
            }

            vc_state_.resize(num_vcs, VCState::IDLE);
            vc_output_port_.resize(num_vcs, Direction::LOCAL);
            vc_output_vc_.resize(num_vcs, 0);
        }

        Direction get_direction() const { return direction_; }
        uint32_t num_vcs() const { return num_vcs_; }

        VCBuffer &get_vc(uint32_t vc_id) { return vc_buffers_[vc_id]; }
        const VCBuffer &get_vc(uint32_t vc_id) const { return vc_buffers_[vc_id]; }

        VCState get_vc_state(uint32_t vc_id) const { return vc_state_[vc_id]; }
        void set_vc_state(uint32_t vc_id, VCState state) { vc_state_[vc_id] = state; }

        static std::string vc_state_to_string(VCState state)
        {
            switch (state)
            {
            case VCState::IDLE:
                return "IDLE";
            case VCState::ROUTING:
                return "ROUTING";
            case VCState::VC_ALLOCATED:
                return "VC_ALLOC";
            case VCState::ACTIVE:
                return "ACTIVE";
            default:
                return "UNKNOWN";
            }
        }

        Direction get_output_port(uint32_t vc_id) const { return vc_output_port_[vc_id]; }
        void set_output_port(uint32_t vc_id, Direction port) { vc_output_port_[vc_id] = port; }

        uint32_t get_output_vc(uint32_t vc_id) const { return vc_output_vc_[vc_id]; }
        void set_output_vc(uint32_t vc_id, uint32_t out_vc) { vc_output_vc_[vc_id] = out_vc; }

        bool has_data() const
        {
            for (const auto &vc : vc_buffers_)
            {
                if (!vc.is_empty())
                    return true;
            }
            return false;
        }

    private:
        Direction direction_;
        uint32_t num_vcs_;
        std::vector<VCBuffer> vc_buffers_;
        std::vector<VCState> vc_state_;
        std::vector<Direction> vc_output_port_;
        std::vector<uint32_t> vc_output_vc_;
    };

    // CREDIT MANAGER

    class CreditManager
    {
    public:
        CreditManager(uint32_t num_output_ports,
                      uint32_t num_vcs_per_port,
                      uint32_t vc_depth,
                      bool enable_trace = false)
            : num_ports_(num_output_ports),
              num_vcs_per_port_(num_vcs_per_port),
              vc_depth_(vc_depth),
              enable_trace_(enable_trace)
        {

            credits_.resize(num_output_ports);
            for (auto &port_credits : credits_)
            {
                port_credits.resize(num_vcs_per_port, vc_depth);
            }
        }

        bool has_credit(Direction port, uint32_t vc_id) const
        {
            uint32_t port_idx = static_cast<uint32_t>(port);
            if (port_idx >= num_ports_ || vc_id >= num_vcs_per_port_)
                return false;
            return credits_[port_idx][vc_id] > 0;
        }

        uint32_t get_credits(Direction port, uint32_t vc_id) const
        {
            uint32_t port_idx = static_cast<uint32_t>(port);
            if (port_idx >= num_ports_ || vc_id >= num_vcs_per_port_)
                return 0;
            return credits_[port_idx][vc_id];
        }

        void consume_credit(Direction port, uint32_t vc_id, uint32_t router_id = 0)
        {
            uint32_t port_idx = static_cast<uint32_t>(port);
            if (port_idx < num_ports_ && vc_id < num_vcs_per_port_)
            {
                if (credits_[port_idx][vc_id] > 0)
                {
                    credits_[port_idx][vc_id]--;

                    if (enable_trace_)
                    {
                        std::cout << "[CREDIT] R" << router_id
                                  << " consumed credit: port=" << dir_to_string(port)
                                  << " vc=" << vc_id
                                  << " remaining=" << credits_[port_idx][vc_id] << "\n";
                    }
                }
            }
        }

        void return_credit(Direction port, uint32_t vc_id, uint32_t router_id = 0)
        {
            uint32_t port_idx = static_cast<uint32_t>(port);
            if (port_idx < num_ports_ && vc_id < num_vcs_per_port_)
            {
                if (credits_[port_idx][vc_id] < vc_depth_)
                {
                    credits_[port_idx][vc_id]++;

                    if (enable_trace_)
                    {
                        std::cout << "[CREDIT] R" << router_id
                                  << " returned credit: port=" << dir_to_string(port)
                                  << " vc=" << vc_id
                                  << " now=" << credits_[port_idx][vc_id] << "\n";
                    }
                }
            }
        }

        void reset()
        {
            for (auto &port_credits : credits_)
            {
                for (auto &credit : port_credits)
                {
                    credit = vc_depth_;
                }
            }
        }

    private:
        uint32_t num_ports_;
        uint32_t num_vcs_per_port_;
        uint32_t vc_depth_;
        bool enable_trace_;
        std::vector<std::vector<uint32_t>> credits_;
    };

    // VC SELECTOR

    class VCSelector
    {
    public:
        VCSelector(uint32_t num_vcs)
            : num_vcs_(num_vcs), last_selected_(0) {}

        // CRITICAL FIX #2: Expose state for round-robin fairness
        uint32_t get_last_selected() const { return last_selected_; }
        void set_last_selected(uint32_t vc) { last_selected_ = vc % num_vcs_; }

        void reset() { last_selected_ = 0; }

    private:
        uint32_t num_vcs_;
        uint32_t last_selected_;
    };

}

#endif