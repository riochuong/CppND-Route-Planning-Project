#pragma once

#include <limits>
#include <cmath>
#include <unordered_map>
#include <iostream>

#include "model.h"

class RouteModel : public Model {

  public:
    class Node : public Model::Node {
      public:
        // Add public Node variables and methods here.
        
        Node(){}
        Node(int idx, RouteModel * search_model, Model::Node node) : Model::Node(node), parent_model(search_model), index(idx) {}
        
        Node *parent = nullptr;
        // heuristic values for A* planner
        float h_value = std::numeric_limits<float>::max();
        float g_value = 0.0;
        bool visited = false;
        std::vector<Node *> neighbors;

        // calucalte distance between this node and other node 
        float distance (const RouteModel::Node &other_node) const {
            return std::sqrt(std::pow(this->x - other_node.x, 2) + std::pow(this->y - other_node.y, 2));
        }
        void FindNeighbors(void);
        
      private:
        // Add private Node variables and methods here.
        int index;
        RouteModel *parent_model = nullptr;
        RouteModel::Node *FindNeighbor (const std::vector<int> node_indices) const;
        
    };

    std::vector<RouteModel::Node> const &SNodes() const { return this->m_Nodes; }
    // primariy for testing purpose 
    auto const &GetNodeToRoadMap() const { return this->node_ro_road; }
    
    // Add public RouteModel variables and methods here.
    RouteModel(const std::vector<std::byte> &xml);  
    std::vector<Node> path; // This variable will eventually store the path that is found by the A* search.
    RouteModel::Node const &FindClosestNode(float x, float y);
  private:
    // Add private RouteModel variables and methods here.
    std::vector<RouteModel::Node> m_Nodes;
    std::unordered_map<int, std::vector<const Model::Road *> > node_ro_road;
    void CreateNodeToRoadHashmap(void);
};
