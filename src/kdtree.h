/* \author Aaron Brown */
// Quiz on implementing kd tree

#pragma once

#include <queue>
#include <cmath>

float sq_dist(const std::vector<float> &v1, const std::vector<float> &v2);

// Structure to represent node of kd tree
struct Node;

struct KdTree {
private:
    const int n_dimensions = 4;
public:
    Node *root;
    KdTree();
    void insert(std::vector<float> point, int id);
    std::vector<int> search(std::vector<float> target, float distanceTol);
};
