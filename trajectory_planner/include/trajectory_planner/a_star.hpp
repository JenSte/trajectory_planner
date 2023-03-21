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
    typename Heuristic,
    typename TotalCostsMap,
    bool UseClosedSet
> class AStar
{
public:

    // Set type to hold all the nodes that were "opened" during the search.
    using NodesSet = std::unordered_set<Node, boost::hash<Node>>;

    // Map type to store the predecessors of nodes.
    using PredecessorsMap = std::unordered_map<Node, Node, boost::hash<Node>>;

    std::vector<Node> search(
        const GoalReached& goal_reached,
        const GetNeighbours& get_neighbours,
        const MovementCost& movement_cost,
        const Heuristic& heuristic,
        const Node& start,
        TotalCostsMap& total_costs)
    {
        // The "open" nodes to process.
        PriorityQueue<Node> open_nodes;
        open_nodes.push(start, 0.0);

        // Set of nodes that were already processed and that will not be looked at
        // again. Only used if the 'UseClosedSet' template argument is true.
        NodesSet closed_set;

        // Maps nodes to their predecessor on the path back to the start node.
        PredecessorsMap predecessors;

        total_costs.set(start, 0.0);

        while (!open_nodes.empty()) {
            Node node = open_nodes.pop();

            if (goal_reached(node)) {
                return reconstruct_path(predecessors, node);
            }

            if (UseClosedSet) {
                closed_set.insert(node);
            }

            for (const Node& neighbour: get_neighbours(node)) {
                if (UseClosedSet) {
                    const auto it = closed_set.find(neighbour);
                    if (it != closed_set.end()) {
                        continue;
                    }
                }

                // The path cost if we would enter the neighbour via the current node.
                double total_cost = total_costs.get(node) + movement_cost(node, neighbour);
                if (total_cost < total_costs.get(neighbour)) {
                    predecessors[neighbour] = node;
                    total_costs.set(neighbour, total_cost);
                    open_nodes.push(neighbour, total_cost + heuristic(neighbour));
                }
            }
        }

        // No path found.
        return std::vector<Node>();
    }

private:

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
};

}

#endif
