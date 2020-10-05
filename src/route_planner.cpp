#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // TODO 2: Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.
    start_node = &model.FindClosestNode(start_x, start_y);  // & is used to get the memory address of the closest node and store it to start_node
    end_node = &model.FindClosestNode(end_x, end_y);
}

// TODO 3: Implement the CalculateHValue method.
// Tips:
// - You can use the distance to the end_node for the h value.
// - Node objects have a distance method to determine the distance to another node.
float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node);   // * is used to access the value of the memory address of end_node
}

// TODO 4: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
// Tips:
// - Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
// - For each node in current_node.neighbors, set the parent, the h_value, the g_value. 
// - Use CalculateHValue below to implement the h-Value calculation.
// - For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.
void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    // FindNeighbors of current_node
    current_node->FindNeighbors();
    for (auto *neighbor : current_node->neighbors)
    {
        neighbor->h_value = CalculateHValue(neighbor);
        neighbor->g_value = current_node->g_value + current_node->distance(*neighbor);
        neighbor->parent = current_node;
        open_list.push_back(neighbor);
        neighbor->visited = true;
    }
}

// TODO 5: Complete the NextNode method to sort the open list and return the next node.
// Tips:
// - Sort the open_list according to the sum of the h value and g value.
// - Create a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer.
static bool CompareNodeValue (const RouteModel::Node* node1, const RouteModel::Node* node2)
{
  float f1 = node1->g_value + node1->h_value; // f1 = g1 + h1
  float f2 = node2->g_value + node2->h_value; // f2 = g2 + h2
  return f1 > f2; 
}

RouteModel::Node *RoutePlanner::NextNode() {
    // sort open_list
    std::sort(RoutePlanner::open_list.begin(), RoutePlanner::open_list.end(), CompareNodeValue);
    // create pointer to next node
    RouteModel::Node *next_node = RoutePlanner::open_list.back();
    // remove next node from open_list
    open_list.pop_back();
    // return the pointer
    return next_node;
}


// TODO 6: Complete the ConstructFinalPath method to return the final path found from your A* search.
// Tips:
// - This method should take the current (final) node as an argument and iteratively follow the 
//   chain of parents of nodes until the starting node is found.
// - For each node in the chain, add the distance from the node to its parent to the distance variable.
// - The returned vector should be in the correct order: the start node should be the first element
//   of the vector, the end node should be the last element.
std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;
    // iterate through the chain of parents until starting node is found
    while (current_node != start_node)
    {
        auto next_node = current_node->parent;
        distance += next_node->distance(*current_node);
        path_found.push_back(*current_node);
        current_node = next_node;
    }
    // ensure to also add the distance of the current_node (start_node) to the
    // most recent added previous node and add it to the path vector
    distance += current_node->distance(path_found.back());
    path_found.push_back(*current_node);

    // flip order of the vector
    std::reverse(path_found.begin(), path_found.end());

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}


// TODO 7: Write the A* Search algorithm here.
// Tips:
// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    std::vector<RouteModel::Node> final_path;

    while (current_node != end_node)
    {
        // AddNeighbors of current node to open_list
        AddNeighbors(current_node);

        // sort open_list and get next node using NextNode()
        current_node = NextNode();
    }
    // when current node = end_node get final path using ConstructFinalPath()
    final_path = ConstructFinalPath(current_node);
       
    // store final path in m_Model.path to display it on the map
    m_Model.path = final_path;
}