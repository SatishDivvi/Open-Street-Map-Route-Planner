#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;
    RouteModel::Node parent;

    while(current_node-> parent != nullptr) {
        path_found.emplace_back(*current_node);
        parent = *(current_node->parent);
        distance += current_node->distance(parent);
        current_node = current_node->parent;
    }
    path_found.emplace_back(*current_node);
    distance *= m_Model.MetricScale();
    return path_found;
}

void RoutePlanner::AStarSearch() {
    end_node->parent = start_node;
    m_Model.path = ConstructFinalPath(end_node);
}

float RoutePlanner::CalculateHValue(RouteModel::Node *node) {
    return node->distance(*end_node);
}

bool Compare(const RouteModel::Node *node1, const RouteModel::Node *node2) {
    float f1 = node1->h_value + node1->g_value;
    float f2 = node2->h_value + node2->g_value;
    if(f1 < f2){
        return true;
    } else {
        return false;
    }
}

RouteModel::Node *RoutePlanner::NextNode() {
    std::sort(open_list.begin(), open_list.end(), Compare);
    RouteModel::Node *current_node = open_list.front();
    open_list.erase(open_list.begin());
    return current_node;
}
