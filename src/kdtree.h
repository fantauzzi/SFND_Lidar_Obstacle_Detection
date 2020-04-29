/* \author Aaron Brown */
// Quiz on implementing kd tree

#pragma once

#include <queue>

float sq_dist(const std::vector<float> &v1, const std::vector<float> &v2);

// Structure to represent node of kd tree
struct Node;

struct KdTree {
    Node *root;
    KdTree();
    void insert(std::vector<float> point, int id);
    std::vector<int> search(std::vector<float> target, float distanceTol);
private:
    // Number of point dimensions; it does not include the intensity channel, even if present.
    const int n_dimensions = 3;
};
