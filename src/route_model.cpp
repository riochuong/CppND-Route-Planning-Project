#include "route_model.h"
#include <iostream>

RouteModel::RouteModel(const std::vector<std::byte> &xml) : Model(xml) {
    int counter = 0;
    for (auto eachNode : this->Nodes()) {
        RouteModel::Node newNode (counter, this, eachNode);
        this->m_Nodes.push_back(newNode);
        counter++;
    }
    // also create node_no_road dict
    CreateNodeToRoadHashmap();
}

void RouteModel::CreateNodeToRoadHashmap(void) {
    /** 
     * Create Hashmap to map node to all the roads that it belongs to
     * @param: N/A
     * @return: N/A
    */
    for (auto &road : this->Roads()) {
        // skip if model is Footway
        if (road.type == Model::Road::Type::Footway) continue;

        for (auto node_idx: Ways()[road.way].nodes) {
            // check to see if we need to create new list 
            if (this->node_ro_road.find(node_idx) == this->node_ro_road.end()) {
                std::vector<const Model::Road *> new_vect;
                new_vect.push_back(&road);
                this->node_ro_road.emplace(node_idx, new_vect);
            }
            else{
                this->node_ro_road[node_idx].push_back(&road);
            }
        }
        
    } 

}

RouteModel::Node *RouteModel::Node::FindNeighbor(const std::vector<int> node_indices) const {
    /**
     * Find the closest neighbor node that is not itself and is not visited yet
     * @param node_indices: list of neighbor nodes
     * @erturn : pointer to the closest node of type RouteModel::Node
     */
    auto &nodes = this->parent_model->SNodes();
    float min_dist = std::numeric_limits<float>::max();
    RouteModel::Node *closest_node = nullptr;
    for (auto idx : node_indices) {
        auto &node = nodes[idx];
        // discard already visited node and same index node
        if (idx == this->index) continue;
        if (node.visited) continue;

        float dist = this->distance(node);
        
        if (dist < min_dist) {
            min_dist = dist;
            closest_node = &node;
        }
    }
    return closest_node;
}

void RouteModel::Node::FindNeighbors(void) {
    /**
     * Populate all neighbors node of this current node to the member vector neighbors
     * Need to go through each road that this node belongs to and add all the neighbors.
     */
    auto roads_of_node = this->parent_model->node_ro_road[this->index];
    for (auto road : roads_of_node) {
        auto node_indices = this->parent_model->Ways()[road->way].nodes;
        RouteModel::Node *closest_node = FindNeighbor(node_indices);
        // push to the neighbors if we found good node
        if (closest_node) this->neighbors.push_back(closest_node);
    }
}

RouteModel::Node &RouteModel::FindClosestNode(float x, float y) {
    /**
     *  Find the closest Node to the given coordinates
     * @param x: x coordinate 
     * @param y: y coordinate
     * @return : reference to node that is closest to the given coordinates.
     * 
     */
    float min_dist = std::numeric_limits<float>::max();
    int closest_idx = -1;
    for (auto &road : this->Roads()) {
        if (road.type == Model::Road::Type::Footway) continue;
        auto node_indices = Ways()[road.way].nodes;
        for (auto node_idx : node_indices) {
            auto node = SNodes()[node_idx];
            float dist = std::sqrt(std::pow(x - node.x, 2) + std::pow(y - node.y, 2));
            if (dist < min_dist) {
                min_dist = dist;
                closest_idx = node_idx;
            }
        }
    }
    return SNodes()[closest_idx];
}