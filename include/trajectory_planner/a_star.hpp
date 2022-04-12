#ifndef TRAJECTORY_PLANNER_A_STAR_HPP
#define TRAJECTORY_PLANNER_A_STAR_HPP

#include <boost/container_hash/hash.hpp>
#include <boost/heap/fibonacci_heap.hpp>

#include <algorithm>
#include <limits>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace trajectory_planner
{

// A priority queue that allows updating the priority value of an element.
template<
    typename Node
> class PriorityQueue
{
    // An item in the heap.
    struct HeapItem
    {
        double priority;

        Node node;

        friend bool operator<(const HeapItem& lh, const HeapItem& rh)
        {
            // The boost::heap library implements max heaps, therefore
            // this comparison operator is "reversed", so we can get
            // the item with the smallest priority out of the heap.
            return lh.priority > rh.priority;
        }
    };

    using heap_type = boost::heap::fibonacci_heap<HeapItem>;

public:

    // Add a node to the priority queue with the given priority, or change
    // the priority of a stored item if the item is already in the queue.
    void push(
        Node node,
        double priority)
    {
        HeapItem item{priority, node};

        auto it = map_.find(node);
        if (it != map_.end()) {
            // This node is already in the heap, update it's priority.
            heap_.update(it->second, item);
        } else {
            // This node is not yet in the heap, add it and store it's handle.
            map_[node] = heap_.push(item);
        }
    }

    // Return the item with the smallest priority value in the queue. The priority
    // queue must not be empty when this function is called.
    Node pop()
    {
        HeapItem item = heap_.top();

        heap_.pop();
        map_.erase(item.node);

        return item.node;
    }

    // Return 'true' if the queue is empty, 'false' otherwise.
    bool empty() const
    {
        return heap_.empty();
    }

private:

    // The heap to store the items.
    heap_type heap_;

    // Maps nodes to heap handles, so that items' priorities can be changed.
    std::unordered_map<Node, typename heap_type::handle_type, boost::hash<Node>> map_;
};

template<
    typename Node,
    typename GoalReached,
    typename GetNeighbours,
    typename MovementCost,
    typename Heuristic
> class AStar
{
public:

    // Set type to hold all the nodes that were "opened" during the search.
    using OpenedNodesSet = std::unordered_set<Node, boost::hash<Node>>;

    // The type returned from the 'search()' function. The first element of the tuple
    // is the path (can be empty if no path is found), while the second element holds
    // all the nodes that the algorithm "opened".
    using search_result = std::tuple<std::vector<Node>, OpenedNodesSet>;

    // Map type to store the predecessors of nodes.
    using PredecessorsMap = std::unordered_map<Node, Node, boost::hash<Node>>;

    // Map type to store the costs of nodes.
    using TotalCostsMap = std::unordered_map<Node, double, boost::hash<Node>>;

    search_result search(
        const GoalReached& goal_reached,
        const GetNeighbours& get_neighbours,
        const MovementCost& movement_cost,
        const Heuristic& heuristic,
        const Node& start)
    {
        // The "open" nodes to process.
        PriorityQueue<Node> open_nodes;
        open_nodes.push(start, 0.0);

        // Maps nodes to their predecessor on the path back to the start node.
        PredecessorsMap predecessors;

        // Maps a node to the cost of the best known path from the start to that node.
        TotalCostsMap total_costs;
        total_costs[start] = 0.0;

        while (!open_nodes.empty()) {
            Node node = open_nodes.pop();

            if (goal_reached(node)) {
                return std::make_tuple(
                    reconstruct_path(predecessors, node),
                    opened_nodes(total_costs));
            }

            for (const Node& neighbour: get_neighbours(node)) {
                // The path cost if we would enter the neighbour via the current node.
                double total_cost =
                    get_total_cost(total_costs, node) + movement_cost(node, neighbour);
                if (total_cost < get_total_cost(total_costs, neighbour)) {
                    predecessors[neighbour] = node;
                    total_costs[neighbour] = total_cost;
                    open_nodes.push(neighbour, total_cost + heuristic(neighbour));
                }
            }
        }

        // No path found.
        return std::make_tuple(std::vector<Node>(), opened_nodes(total_costs));
    }

private:

    // Return the total cost in the map, or infinity if the node is not in the map.
    double get_total_cost(
        const TotalCostsMap& total_costs,
        const Node& node)
    {
        const auto it = total_costs.find(node);
        if (it == total_costs.end()) {
            return std::numeric_limits<double>::infinity();
        }

        return it->second;
    }

    std::vector<Node> reconstruct_path(
        const PredecessorsMap& predecessors,
        const Node& end_node)
    {
        std::vector<Node> result;
        result.push_back(end_node);

        for (;;) {
            const auto it = predecessors.find(result.back());
            if (it == predecessors.end()) {
                // Done, we reached the start of the path.
                break;
            }

            result.push_back(it->second);
        }

        std::reverse(std::begin(result), std::end(result));
        return result;
    }

    // Create a set that contains all nodes that the algorithm looked at.
    OpenedNodesSet opened_nodes(
        const TotalCostsMap& total_costs)
    {
        OpenedNodesSet result;

        std::transform(
            total_costs.begin(),
            total_costs.end(),
            std::inserter(result, result.end()),
            [](const auto& pair){ return pair.first; });

        return result;
    }
};

}

#endif
