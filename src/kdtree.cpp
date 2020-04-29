/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "kdtree.h"
#include <queue>
#include <cmath>

/** Returns the square Euclidean distance between two points. Points must have at least 3 dimensions; any dimension
 * after the 3rd is ignored (e.g. the I in an RGBI point).
 * @param v1 the first given point.
 * @param v2 the second given point.
 * @return the square of their distance.
 */
float sq_dist(const std::vector<float> &v1, const std::vector<float> &v2) {
    float diff_x = v1[0] - v2[0];
    float diff_y = v1[1] - v2[1];
    float diff_z = v1[2] - v2[2];
    float res = diff_x * diff_x + diff_y * diff_y + diff_z * diff_z;
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

/** Insert a new node in the right place in the KD-tree
 * @param point  The point to be inserted; it may have more than 3 dimensions, but only the first 3 dimensions will
 * be considered.
 * @param id The id for the new point.
 */
void KdTree::insert(std::vector<float> point, int id) {
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

/** Return a vector of point ids in the tree that are within a given distance of target.
 * @param target the target node.
 * @param distanceTol the given distance.
 * @return the ids of the selected points.
 */
std::vector<int> KdTree::search(std::vector<float> target, float distanceTol) {
    const float sq_distanceTol = distanceTol * distanceTol;
    // Will collect the result, to be returned
    std::vector<int> ids;
    /* Queue of nodes that are candidate to be in the same cluster as `target`, to be checked; each node is stored
     * in the queue in a pair with an integer, which is the depth of the node in the tree (0 for the root) */
    std::queue<std::pair<Node *, int>> pending;
    pending.emplace(std::make_pair(root, 0)); // Start with the root, which is at depth 0 in the tree
    while (!pending.empty()) {
        // Pop from the queue the next tree node to be processed
        auto item = pending.front();
        pending.pop();
        Node *pCurrent = item.first;
        int dimension = item.second;
        // Check if the node is in the cluster, in case add its id to `ids`
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

