#include "processing.h"
#include "kdtree.h"


std::vector<std::vector<int>>
euclideanCluster(const std::vector<std::vector<float>> &points, KdTree *tree, float distanceTol) {

    // DONE: Fill out this function to return list of indices for each cluster

    // Will collect the result
    std::vector<std::vector<int>> clusters;
    // Keeps note for each node if already processed
    std::vector<bool> processed(points.size(), false);

    for (int point_id = 0; point_id < points.size(); ++point_id) {
        // Skip the point if already processed
        if (processed[point_id])
            continue;
        /* Queue of ids of points in the same cluster as the current point (cluster_id);
         * invariant: it only contains points not yet processed (not in processed_points) */
        std::set<int> pending;
        pending.emplace(point_id);
        std::vector<int> cluster;
        while (!pending.empty()) {
            // Pop one, mark it as processed and add it to the current cluster
            auto processing_id = *pending.begin();
            pending.erase(pending.begin());
            processed[processing_id] = true;
            cluster.emplace_back(processing_id);
            // Find all ids of nodes that are within distance tolerance from it
            auto neighbors = tree->search(points[processing_id], distanceTol);
            // If not yet processed, schedule them for processing
            for (const auto &neighbor: neighbors)
                if (! processed[neighbor])
                    pending.emplace(neighbor);
        }
        // Append the just built cluster to the sequence of clusters that will be returned
        clusters.emplace_back(cluster);
    }

    return clusters;
}
