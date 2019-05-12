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
        std::vector<Node> *neighbors = nullptr;
      private:
        // Add private Node variables and methods here.
        int index;
        RouteModel * parent_model = nullptr;
    };

    std::vector<RouteModel::Node> const &SNodes() const { return this->m_Nodes; }
    
    // Add public RouteModel variables and methods here.
    RouteModel(const std::vector<std::byte> &xml);  
    std::vector<Node> path; // This variable will eventually store the path that is found by the A* search.

  private:
    // Add private RouteModel variables and methods here.
    std::vector<RouteModel::Node> m_Nodes;

};
