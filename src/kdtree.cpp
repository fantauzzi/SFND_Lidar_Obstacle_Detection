/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "kdtree.h"
#include <queue>
#include <cmath>

float sq_dist(const std::vector<float> &v1, const std::vector<float> &v2) {
    float diff_x = v1[0]-v2[0];
    float diff_y = v1[1]-v2[1];
    float diff_z = v1[2]-v2[2];
    float res = diff_x*diff_x+diff_y*diff_y+diff_z*diff_z;
    return res;
}

// Structure to represent node of kd tree
struct Node {
    std::vector<float> point;
    int id;
    Node *left;
    Node *right;

    Node(std::vector<float> arr, int setId)
            : point(arr), id(setId), left(NULL), right(NULL) {}
};


KdTree::KdTree() : root(NULL) {};

void KdTree::insert(std::vector<float> point, int id) {
    // DONE: Fill in this function to insert a new point into the tree
    // the function should create a new node and place correctly with in the root
    auto pToBeInserted = new Node(point, id);
    if (!root)
        root = pToBeInserted;
    else {
        auto pCurrent = root;
        int switchOnComponent = -1;
        while (true) {
            switchOnComponent = (switchOnComponent + 1) % n_dimensions;
            if (pToBeInserted->point[switchOnComponent] <= pCurrent->point[switchOnComponent]) {
                if (!pCurrent->left) {
                    pCurrent->left = pToBeInserted;
                    break;
                }
                pCurrent = pCurrent->left;
            } else {
                if (!pCurrent->right) {
                    pCurrent->right = pToBeInserted;
                    break;
                }
                pCurrent = pCurrent->right;
            }
        }
    }
}

// return a list of point ids in the tree that are within distance of target
std::vector<int> KdTree::search(std::vector<float> target, float distanceTol) {
    const float sq_distanceTol = distanceTol * distanceTol;
    std::vector<int> ids;
    std::queue<std::pair<Node *, int>> pending;
    pending.emplace(std::make_pair(root, 0));
    while (!pending.empty()) {
        // Fetch the next tree node to be processed
        auto item = pending.front();
        pending.pop();
        Node *pCurrent = item.first;
        int dimension = item.second;
        // Check if the node is in the cluster
        if (abs(pCurrent->point[dimension] - target[dimension]) <= distanceTol)
            if (sq_dist(pCurrent->point, target) <= sq_distanceTol)
                ids.emplace_back(pCurrent->id);
        // Consider the children, schedule them for processing or prune them as necessary
        int childDim = (dimension + 1) % n_dimensions;
        auto pLeft = pCurrent->left;
        if (pLeft && pCurrent->point[dimension] >= target[dimension] - distanceTol)
            pending.emplace(std::make_pair(pLeft, childDim));
        auto pRight = pCurrent->right;
        if (pRight && pCurrent->point[dimension] <= target[dimension] + distanceTol)
            pending.emplace(std::make_pair(pRight, childDim));
    }

    return ids;
}

