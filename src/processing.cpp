#include "processing.h"
#include "kdtree.h"
#include <unordered_set>
#include <random>
#include <cassert>
#include <cmath>


std::vector<std::vector<int>>
euclideanCluster(const std::vector<std::vector<float>> &points, KdTree *tree, float distanceTol, int minSize = 0) {

    // Will collect the result
    std::vector<std::vector<int>> clusters;
    // Keeps note for each node if already processed
    std::vector<bool> processed(points.size(), false);

    for (int point_id = 0; point_id < points.size(); ++point_id) {
        // Skip the point if already processed
        if (processed[point_id])
            continue;
        /* Set of ids of points in the same cluster as the current point (cluster_id);
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
                if (!processed[neighbor])
                    pending.emplace(neighbor);
        }
        // Append the just built cluster to the sequence of clusters that will be returned
        if (cluster.size() >= minSize)
            clusters.emplace_back(cluster);
    }

    return clusters;
}

std::pair<typename pcl::PointCloud<pcl::PointXYZI>::Ptr, typename pcl::PointCloud<pcl::PointXYZI>::Ptr>
Ransac(const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud, int maxIterations, float distanceTol) {

    // srand(time(NULL));

    // Used to sample points from the cloud
    std::default_random_engine generator;
    std::uniform_int_distribution<int> distribution(0, cloud->size() - 1);

    /* Keep a set with the best result so far, and a separate working set. Will swap between the two when the
     * working set turns out to be better than the other set; it prevents unnecessary copying of the set.
     * Note: an alternative viable implementation is to keep track of the best set so far by storing only its
     * two/three representative points, as opposed to the whole set of inliners. */
    std::vector<int> inliersResult1;
    std::vector<int> inliersResult2;
    std::vector<int> &working = inliersResult1;
    std::vector<int> &best = inliersResult2;

    for (int iter = 0; iter < maxIterations; ++iter) {
        // std::cout << "Iteration# " << iter << std::endl;
        // Fetch three random points from the cloud, without repetition (repetition would produce a division by zero below)
        int pick = distribution(generator);
        int second_pick = pick;
        int third_pick = pick;
        while (second_pick == pick)
            second_pick = distribution(generator);
        while (third_pick == pick || third_pick == second_pick)
            third_pick = distribution(generator);

        float x1 = cloud->points[pick].x;
        float y1 = cloud->points[pick].y;
        float z1 = cloud->points[pick].z;
        float x2 = cloud->points[second_pick].x;
        float y2 = cloud->points[second_pick].y;
        float z2 = cloud->points[second_pick].z;
        float x3 = cloud->points[third_pick].x;
        float y3 = cloud->points[third_pick].y;
        float z3 = cloud->points[third_pick].z;
        float i = (y2 - y1) * (z3 - z1) - (z2 - z1) * (y3 - y1);
        float j = (z2 - z1) * (x3 - x1) - (x2 - x1) * (z3 - z1);
        float k = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);
        float a = i;
        float b = j;
        float c = k;
        float d = -(i * x1 + j * y1 + k * z1);
        float denominator = std::sqrt(a * a + b * b + c * c);
        // Build the vector of inliners for the two sample points and the given max distance
        // for (int point_idx = 0; point_idx < cloud->size(); ++point_idx) {
        int point_idx = -1;
        for (auto point: cloud->points) {
            ++point_idx;
            // pcl::PointXYZI point = (*cloud)[point_idx];
            float dist = std::abs(a * point.x + b * point.y + c * point.z + d) / denominator;
            if (dist <= distanceTol)
                working.emplace_back(point_idx);
        }
        // Keep track of the best (i.e. biggest) set of inliners so far
        if (working.size() > best.size()) {
            std::cout << "RANSAC-1: obstacle points/total points = " << static_cast<float>(cloud->size()-working.size())/cloud->size() << std::endl;
            std::swap(working, best);
        }
        working.clear();
    }
    std::cout << "RANSAC-2: obstacle points/total points = " << static_cast<float>(cloud->size()-best.size())/cloud->size() << std::endl;

    // Return the two clouds, with inliers and outliers
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    inliers->indices = best;
    std::pair<typename pcl::PointCloud<pcl::PointXYZI>::Ptr, typename pcl::PointCloud<pcl::PointXYZI>::Ptr>  pair_of_clouds(separateClouds<typename pcl::PointXYZI>(inliers, cloud));
    assert(cloud->size() == pair_of_clouds.first->size()+pair_of_clouds.second->size());
    return pair_of_clouds;
}
