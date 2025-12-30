#ifndef GPU_NOC_ROUTING_HPP
#define GPU_NOC_ROUTING_HPP

#include "gpu_noc_types.hpp"
#include <stdexcept>

namespace gpu_noc
{

    class RoutingEngine
    {
    public:
        explicit RoutingEngine(const NoC_Config &config)
            : config_(config) {}

        Direction compute_route(const Flit &flit, uint32_t current_router_id) const
        {
            if (!flit.is_head())
            {
                throw std::runtime_error("Routing computation called for non-HEAD flit");
            }

            uint32_t cur_x, cur_y;
            config_.router_id_to_xy(current_router_id, cur_x, cur_y);

            if (config_.xy_routing)
            {
                return compute_xy_route(cur_x, cur_y, flit.dst_x, flit.dst_y);
            }
            else
            {
                return compute_yx_route(cur_x, cur_y, flit.dst_x, flit.dst_y);
            }
        }

        bool is_route_valid(Direction dir, uint32_t current_router_id) const
        {
            uint32_t cur_x, cur_y;
            config_.router_id_to_xy(current_router_id, cur_x, cur_y);

            switch (dir)
            {
            case Direction::NORTH:
                return cur_y > 0;
            case Direction::SOUTH:
                return cur_y + 1 < config_.mesh_height;
            case Direction::EAST:
                return cur_x + 1 < config_.mesh_width;
            case Direction::WEST:
                return cur_x > 0;
            case Direction::LOCAL:
                return true;
            default:
                return false;
            }
        }

        uint32_t get_neighbor_router_id(uint32_t current_router_id, Direction dir) const
        {
            uint32_t cur_x, cur_y;
            config_.router_id_to_xy(current_router_id, cur_x, cur_y);

            uint32_t next_x = cur_x;
            uint32_t next_y = cur_y;

            switch (dir)
            {
            case Direction::NORTH:
                if (cur_y > 0)
                    next_y = cur_y - 1;
                else
                    throw std::runtime_error("Cannot move NORTH from top edge");
                break;
            case Direction::SOUTH:
                if (cur_y + 1 < config_.mesh_height)
                    next_y = cur_y + 1;
                else
                    throw std::runtime_error("Cannot move SOUTH from bottom edge");
                break;
            case Direction::EAST:
                if (cur_x + 1 < config_.mesh_width)
                    next_x = cur_x + 1;
                else
                    throw std::runtime_error("Cannot move EAST from right edge");
                break;
            case Direction::WEST:
                if (cur_x > 0)
                    next_x = cur_x - 1;
                else
                    throw std::runtime_error("Cannot move WEST from left edge");
                break;
            case Direction::LOCAL:
                return current_router_id;
            default:
                throw std::runtime_error("Unknown direction");
            }

            return config_.xy_to_router_id(next_x, next_y);
        }

        uint32_t manhattan_distance(uint32_t src_router_id, uint32_t dst_router_id) const
        {
            uint32_t src_x, src_y, dst_x, dst_y;
            config_.router_id_to_xy(src_router_id, src_x, src_y);
            config_.router_id_to_xy(dst_router_id, dst_x, dst_y);

            return abs_diff(src_x, dst_x) + abs_diff(src_y, dst_y);
        }

        bool is_at_destination(const Flit &flit, uint32_t current_router_id) const
        {
            return flit.dst_router_id == current_router_id;
        }

    private:
        const NoC_Config &config_;

        Direction compute_xy_route(uint32_t cur_x, uint32_t cur_y,
                                   uint32_t dst_x, uint32_t dst_y) const
        {
            if (cur_x < dst_x)
                return Direction::EAST;
            if (cur_x > dst_x)
                return Direction::WEST;
            if (cur_y < dst_y)
                return Direction::SOUTH;
            if (cur_y > dst_y)
                return Direction::NORTH;
            return Direction::LOCAL;
        }

        Direction compute_yx_route(uint32_t cur_x, uint32_t cur_y,
                                   uint32_t dst_x, uint32_t dst_y) const
        {
            if (cur_y < dst_y)
                return Direction::SOUTH;
            if (cur_y > dst_y)
                return Direction::NORTH;
            if (cur_x < dst_x)
                return Direction::EAST;
            if (cur_x > dst_x)
                return Direction::WEST;
            return Direction::LOCAL;
        }

        uint32_t abs_diff(uint32_t a, uint32_t b) const
        {
            return a > b ? (a - b) : (b - a);
        }
    };

}

#endif