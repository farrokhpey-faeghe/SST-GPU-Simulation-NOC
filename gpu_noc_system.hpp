#ifndef GPU_NOC_SYSTEM_HPP
#define GPU_NOC_SYSTEM_HPP

#include "gpu_noc_types.hpp"
#include "gpu_noc_network.hpp"
#include <memory>
#include <unordered_map>
#include <vector>
#include <iostream>
#include <functional>

namespace gpu_noc
{

    // DualNetwork (request+respon)

    class GPUNoCSystem
    {
    public:
        // Configuration structure for system-level parameters
        struct SystemConfig
        {
            // Topology
            uint32_t mesh_width = 4;
            uint32_t mesh_height = 4;

            // GPU Architecture
            uint32_t num_shader_cores = 16;
            uint32_t num_l2_banks = 8;
            uint32_t num_memory_controllers = 4;

            // NoC Parameters
            uint32_t num_vcs_per_port = 2;
            uint32_t vc_buffer_depth = 4;
            uint32_t link_latency = 1;
            uint32_t credit_latency = 1;
            uint32_t flit_size_bytes = 32;
            uint32_t max_packet_size_flits = 4;

            // Routing
            bool use_xy_routing = true;

            // Debug
            bool enable_debug_trace = false;
            bool enable_credit_trace = false;
        };

        explicit GPUNoCSystem(const SystemConfig &sys_config)
            : sys_config_(sys_config)
        {

            // Convert SystemConfig to NoC_Config
            noc_config_.mesh_width = sys_config.mesh_width;
            noc_config_.mesh_height = sys_config.mesh_height;
            noc_config_.num_routers = sys_config.mesh_width * sys_config.mesh_height;

            noc_config_.num_shader_cores = sys_config.num_shader_cores;
            noc_config_.num_l2_banks = sys_config.num_l2_banks;
            noc_config_.num_memory_controllers = sys_config.num_memory_controllers;

            noc_config_.num_vcs_per_port = sys_config.num_vcs_per_port;
            noc_config_.vc_buffer_depth = sys_config.vc_buffer_depth;
            noc_config_.router_pipeline_stages = 4;
            noc_config_.link_latency = sys_config.link_latency;
            noc_config_.credit_latency = sys_config.credit_latency;

            noc_config_.flit_size_bytes = sys_config.flit_size_bytes;
            noc_config_.max_packet_size_flits = sys_config.max_packet_size_flits;
            noc_config_.xy_routing = sys_config.use_xy_routing;

            noc_config_.enable_debug_trace = sys_config.enable_debug_trace;
            noc_config_.enable_credit_trace = sys_config.enable_credit_trace;

            // Validate configuration
            if (!noc_config_.validate())
            {
                throw std::runtime_error("Invalid NoC configuration");
            }

            //  Use DualNetwork, not MeshInterconnect
            dual_network_ = std::make_unique<DualNetwork>(noc_config_);

            // Initialize packet ID counter
            next_packet_id_ = 0;
            current_cycle_ = 0;

            std::cout << "\n========================================\n";
            std::cout << "GPU NoC System Initialized\n";
            std::cout << "========================================\n";
            std::cout << "Topology: " << noc_config_.mesh_width << "x"
                      << noc_config_.mesh_height << " mesh\n";
            std::cout << "Shader Cores: " << noc_config_.num_shader_cores << "\n";
            std::cout << "L2 Banks: " << noc_config_.num_l2_banks << "\n";
            std::cout << "VCs: " << noc_config_.num_vcs_per_port
                      << " x " << noc_config_.vc_buffer_depth << " depth\n";
            std::cout << "Link Latency: " << noc_config_.link_latency << " cycles\n";
            std::cout << "Max Packet Size: " << noc_config_.max_packet_size_flits << " flits\n";
            std::cout << "========================================\n\n";
        }

        // HIGH-LEVEl API: Memory Request Functions

        // Send a read request from SM to L2
        bool send_read_request(uint32_t sm_id, uint32_t l2_id, uint64_t address)
        {
            if (sm_id >= noc_config_.num_shader_cores)
            {
                std::cerr << "[ERROR] Invalid SM ID: " << sm_id << "\n";
                return false;
            }
            if (l2_id >= noc_config_.num_l2_banks)
            {
                std::cerr << "[ERROR] Invalid L2 ID: " << l2_id << "\n";
                return false;
            }

            uint64_t packet_id = next_packet_id_++;

            // DualNetwork API
            bool success = dual_network_->send_read_request(sm_id, l2_id, address, packet_id);

            if (success && sys_config_.enable_debug_trace)
            {
                std::cout << "[SYS] Sent READ_REQ: SM[" << sm_id << "] → L2[" << l2_id
                          << "] addr=0x" << std::hex << address << std::dec
                          << " pkt=" << packet_id << "\n";
            }

            return success;
        }

        // Send a write request from SM to L2
        bool send_write_request(uint32_t sm_id, uint32_t l2_id, uint64_t address,
                                const std::vector<uint8_t> &data)
        {
            if (sm_id >= noc_config_.num_shader_cores)
            {
                std::cerr << "[ERROR] Invalid SM ID: " << sm_id << "\n";
                return false;
            }
            if (l2_id >= noc_config_.num_l2_banks)
            {
                std::cerr << "[ERROR] Invalid L2 ID: " << l2_id << "\n";
                return false;
            }

            uint64_t packet_id = next_packet_id_++;

            bool success = dual_network_->send_write_request(sm_id, l2_id, address,
                                                             packet_id, data);

            if (success && sys_config_.enable_debug_trace)
            {
                std::cout << "[SYS] Sent WRITE_REQ: SM[" << sm_id << "] → L2[" << l2_id
                          << "] addr=0x" << std::hex << address << std::dec
                          << " pkt=" << packet_id << " bytes=" << data.size() << "\n";
            }

            return success;
        }

        // Send a read response from L2 to SM (multi-flit)
        bool send_read_response(uint32_t l2_id, uint32_t sm_id, uint64_t packet_id,
                                const std::vector<uint8_t> &data)
        {
            if (sm_id >= noc_config_.num_shader_cores)
            {
                std::cerr << "[ERROR] Invalid SM ID: " << sm_id << "\n";
                return false;
            }
            if (l2_id >= noc_config_.num_l2_banks)
            {
                std::cerr << "[ERROR] Invalid L2 ID: " << l2_id << "\n";
                return false;
            }

            // CRITICAL FIX (GAP A): Use DualNetwork API (multi-flit)
            bool success = dual_network_->send_read_response(l2_id, sm_id, packet_id, data);

            if (success && sys_config_.enable_debug_trace)
            {
                std::cout << "[SYS] Sent READ_RESP: L2[" << l2_id << "] → SM[" << sm_id
                          << "] pkt=" << packet_id << " bytes=" << data.size() << "\n";
            }

            return success;
        }

        // Send a write response from L2 to SM (acknowledgment)
        bool send_write_response(uint32_t l2_id, uint32_t sm_id, uint64_t packet_id)
        {
            if (sm_id >= noc_config_.num_shader_cores)
            {
                std::cerr << "[ERROR] Invalid SM ID: " << sm_id << "\n";
                return false;
            }
            if (l2_id >= noc_config_.num_l2_banks)
            {
                std::cerr << "[ERROR] Invalid L2 ID: " << l2_id << "\n";
                return false;
            }

            bool success = dual_network_->send_write_response(l2_id, sm_id, packet_id);

            if (success && sys_config_.enable_debug_trace)
            {
                std::cout << "[SYS] Sent WRITE_RESP: L2[" << l2_id << "] → SM[" << sm_id
                          << "] pkt=" << packet_id << "\n";
            }

            return success;
        }

        // Advance simulation by one cycle
        void tick()
        {

            dual_network_->cycle();

            auto sm_arrivals = dual_network_->collect_sm_arrivals();
            auto l2_arrivals = dual_network_->collect_l2_arrivals();

            // Process SM arrivals (responses from L2)
            for (const auto &flit : sm_arrivals)
            {
                process_sm_arrival(flit);
            }

            // Process L2 arrivals (requests from SM)
            for (const auto &flit : l2_arrivals)
            {
                process_l2_arrival(flit);
            }

            current_cycle_++;
        }

        void run_cycles(uint64_t num_cycles)
        {
            for (uint64_t i = 0; i < num_cycles; i++)
            {
                tick();
            }
        }
        // CALLBACK REGISTRATION

        using SMCallback = std::function<void(uint32_t sm_id, const Flit &flit)>;
        using L2Callback = std::function<void(uint32_t l2_id, const Flit &flit)>;

        void register_sm_callback(SMCallback callback)
        {
            sm_callback_ = callback;
        }

        void register_l2_callback(L2Callback callback)
        {
            l2_callback_ = callback;
        }

        void print_statistics() const
        {
            dual_network_->print_statistics();

            std::cout << "\n========================================\n";
            std::cout << "System Statistics\n";
            std::cout << "========================================\n";
            std::cout << "Total Cycles: " << current_cycle_ << "\n";
            std::cout << "Packets Generated: " << next_packet_id_ << "\n";
            std::cout << "SM Arrivals: " << total_sm_arrivals_ << "\n";
            std::cout << "L2 Arrivals: " << total_l2_arrivals_ << "\n";
            std::cout << "========================================\n\n";
        }

        void reset_statistics()
        {
            total_sm_arrivals_ = 0;
            total_l2_arrivals_ = 0;
        }

        // ACCESSORS

        uint64_t get_current_cycle() const { return current_cycle_; }
        uint64_t get_packet_count() const { return next_packet_id_; }
        const NoC_Config &get_noc_config() const { return noc_config_; }

        // Get network statistics
        const NoC_Stats &get_request_network_stats() const
        {
            return dual_network_->get_request_network_stats();
        }

        const NoC_Stats &get_response_network_stats() const
        {
            return dual_network_->get_response_network_stats();
        }

    private:
        SystemConfig sys_config_;
        NoC_Config noc_config_;

        // CRITICAL FIX (GAP A): Use DualNetwork
        std::unique_ptr<DualNetwork> dual_network_;

        // System state
        uint64_t next_packet_id_;
        uint64_t current_cycle_;

        // Statistics
        uint64_t total_sm_arrivals_ = 0;
        uint64_t total_l2_arrivals_ = 0;

        // Callbacks
        SMCallback sm_callback_;
        L2Callback l2_callback_;
        // INTERNAL PROCESSING

        void process_sm_arrival(const Flit &flit)
        {
            total_sm_arrivals_++;

            if (sys_config_.enable_debug_trace)
            {
                std::cout << "[SYS] SM[" << flit.dst_endpoint_id << "] received "
                          << flit_type_to_string(flit.flit_type)
                          << " pkt=" << flit.packet_id
                          << " flit=" << flit.flit_id << "\n";
            }

            // Call registered callback if available
            if (sm_callback_)
            {
                sm_callback_(flit.dst_endpoint_id, flit);
            }
        }

        void process_l2_arrival(const Flit &flit)
        {
            total_l2_arrivals_++;

            if (sys_config_.enable_debug_trace)
            {
                std::cout << "[SYS] L2[" << flit.dst_endpoint_id << "] received "
                          << flit_type_to_string(flit.flit_type)
                          << " pkt=" << flit.packet_id
                          << " flit=" << flit.flit_id << "\n";
            }

            // Call registered callback if available
            if (l2_callback_)
            {
                l2_callback_(flit.dst_endpoint_id, flit);
            }
        }
    };

}

#endif