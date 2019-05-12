#include "route_model.h"
#include <iostream>

RouteModel::RouteModel(const std::vector<std::byte> &xml) : Model(xml) {
    int counter = 0;
    for (auto eachNode : this->Nodes()) {
        RouteModel::Node newNode (counter, this, eachNode);
        this->m_Nodes.push_back(newNode);
        counter++;
    }
}