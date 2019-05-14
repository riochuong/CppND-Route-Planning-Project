#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // scale float coordinates to percentage 
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // find closest point 
    RouteModel::Node start_node = model.FindClosestNode(start_x, start_y);
    RouteModel::Node end_node = model.FindClosestNode(end_x, end_y);
    this->m_Model = model;
    this->start_node = &start_node;
    this->end_node = &end_node;
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    /**
     * Construct the final path in reversed order as the rendering code expect it
     * @param current_node: destination node
     * @return: vector of all node that formed the path in reversed order
     */ 
    std::vector<RouteModel::Node> path_found;
    while(current_node) {
        path_found.push_back(*current_node);
        this->distance += current_node->distance(*current_node->parent);
        current_node = current_node->parent;
    }
    this->distance *= this->m_Model.MetricScale();
    return path_found;
}

void RoutePlanner::AStarSearch(void) {
    this->start_node->visited = true;
    this->open_list.push_back(start_node);
    RouteModel::Node *current_node = nullptr;
    while(!open_list.empty()) {
        current_node = NextNode();
        if (current_node->distance(*this->end_node) == 0.0) {
            this->m_Model.path = ConstructFinalPath(current_node);
            return;
        }
        this->AddNeighbors(current_node);
    }
}

float RoutePlanner::CalculateHValue (const RouteModel::Node *node) {
    /**
     * Calculate Heuristic value of a node based on its euclidean distance to the end node
     * @param node: current node
     * @return : heuristic value of the node 
     */
    return node->distance(*this->end_node);
}

 bool NodeComparator(RouteModel::Node* a, RouteModel::Node* b) {
    // Comparator to sort open list 
        return (a->g_value + a->h_value) < (b->g_value + b->h_value);
}

RouteModel::Node *RoutePlanner::NextNode(void) {
    /**
     * Return the next best node from the open_list and remove it from the list
     * @return: pointer to the next good node based on heuristic and distance
     */
   std::sort(open_list.begin(), open_list.end(), NodeComparator);
   RouteModel::Node *good_node = open_list.front();
   open_list.erase(open_list.begin());
   return good_node;
}

void RoutePlanner::AddNeighbors(RouteModel::Node *node) {
    // populate neighbors of the node
    node->FindNeighbors();
    for (auto &neighbor_node : node->neighbors) {
        neighbor_node->parent = node;
        neighbor_node->g_value = node->g_value + node->distance(*neighbor_node);
        neighbor_node->h_value = CalculateHValue(neighbor_node);
        neighbor_node->visited = true;
        this->open_list.push_back(neighbor_node);
    }
}
