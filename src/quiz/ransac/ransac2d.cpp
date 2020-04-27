/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"
#include <random>
#include <cmath>
#include <algorithm>

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData() {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    // Add inliers
    float scatter = 0.6;
    for (int i = -5; i < 5; i++) {
        double rx = 2 * (((double) rand() / (RAND_MAX)) - 0.5);
        double ry = 2 * (((double) rand() / (RAND_MAX)) - 0.5);
        pcl::PointXYZ point;
        point.x = i + scatter * rx;
        point.y = i + scatter * ry;
        point.z = 0;

        cloud->points.push_back(point);
    }
    // Add outliers
    int numOutliers = 10;
    while (numOutliers--) {
        double rx = 2 * (((double) rand() / (RAND_MAX)) - 0.5);
        double ry = 2 * (((double) rand() / (RAND_MAX)) - 0.5);
        pcl::PointXYZ point;
        point.x = 5 * rx;
        point.y = 5 * ry;
        point.z = 0;

        cloud->points.push_back(point);

    }
    cloud->width = cloud->points.size();
    cloud->height = 1;

    return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D() {
    ProcessPointClouds<pcl::PointXYZ> pointProcessor;
    return pointProcessor.loadPcd("/home/fanta/workspace/SFND_Lidar_Obstacle_Detection/src/sensors/data/pcd");
    // return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene() {
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("2D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->initCameraParameters();
    viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
    viewer->addCoordinateSystem(1.0);
    return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol) {
    srand(time(NULL));

    // DONE: Fill in this function

    // Used to sample points from the cloud
    std::default_random_engine generator;
    std::uniform_int_distribution<int> distribution(0, cloud->size() - 1);

    /* Keep a set with the best result so far, and a separate working set. Will swap between the two when the
     * working set turns out to be better than the other set; it prevents unnecessary copying of the set.
     * Note: an alternative viable implementation is to keep track of the best set so far by storing only its
     * two representative points, as opposed to the whole set of inliners. */
    std::unordered_set<int> inliersResult1;
    std::unordered_set<int> inliersResult2;
    std::unordered_set<int> &working = inliersResult1;
    std::unordered_set<int> &best = inliersResult2;

    for (int iter = 0; iter < maxIterations; ++iter) {
        std::cout << "Iteration# " << iter << std::endl;
        // Fetch two random points from the cloud, without repetition (repetition would produce a division by zero below)
        int pick = distribution(generator);
        int second_pick = pick;
        int third_pick = pick;
        while (second_pick == pick)
            second_pick = distribution(generator);
        while (third_pick == pick || third_pick == second_pick)
            third_pick = distribution(generator);

        auto x1 = cloud->points[pick].x;
        auto y1 = cloud->points[pick].y;
        auto z1 = cloud->points[pick].z;
        auto x2 = cloud->points[second_pick].x;
        auto y2 = cloud->points[second_pick].y;
        auto z2 = cloud->points[second_pick].z;
        auto x3 = cloud->points[third_pick].x;
        auto y3 = cloud->points[third_pick].y;
        auto z3 = cloud->points[third_pick].z;
        auto i = (y2 - y1) * (z3 - z1) - (z2 - z1) * (y3 - y1);
        auto j = (z2 - z1) * (x3 - x1) - (x2 - x1) * (z3 - z1);
        auto k = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);
        auto a = i;
        auto b = j;
        auto c = k;
        auto d = -(i * x1 + j * y1 + k * z1);
        auto denominator = sqrt(a * a + b * b + c * c);
        // Build the set of inliners for the two sample points and the given max distance
        for (int point_idx = 0; point_idx < cloud->size(); ++point_idx) {
            auto point = (*cloud)[point_idx];
            auto dist = abs(a * point.x + b * point.y + c * point.z + d) / denominator;
            if (dist <= distanceTol)
                working.emplace(point_idx);
        }
        // Keep track of the best (iter.e. biggest) set of inliners so far
        if (working.size() > best.size())
            std::swap(working, best);
        working.clear();
    }
    return best;

}

std::unordered_set<int> RansacLine(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol) {
    srand(time(NULL));

    // DONE: Fill in this function

    // Used to sample points from the cloud
    std::default_random_engine generator;
    std::uniform_int_distribution<int> distribution(0, cloud->size() - 1);

    /* Keep a set with the best result so far, and a separate working set. Will swap between the two when the
     * working set turns out to be better than the other set; it prevents unnecessary copying of the set.
     * Note: an alternative viable implementation is to keep track of the best set so far by storing only its
     * two representative points, as opposed to the whole set of inliners. */
    std::unordered_set<int> inliersResult1;
    std::unordered_set<int> inliersResult2;
    std::unordered_set<int> &working = inliersResult1;
    std::unordered_set<int> &best = inliersResult2;

    for (int i = 0; i < maxIterations; ++i) {
        // Fetch two random points from the cloud, without repetition (repetition would produce a division by zero below)
        int pick = distribution(generator);
        int second_pick = pick;
        while (second_pick == pick)
            second_pick = distribution(generator);
        auto x1 = cloud->points[pick].x;
        auto y1 = cloud->points[pick].y;
        auto x2 = cloud->points[second_pick].x;
        auto y2 = cloud->points[second_pick].y;
        auto a = y1 - y2;
        auto b = x2 - x1;
        auto c = x1 * y2 - x2 * y1;
        auto denominator = sqrt(a * a + b * b);
        // Build the set of inliners for the two sample points and the given max distance
        for (int j = 0; j < cloud->size(); ++j) {
            auto point = (*cloud)[j];
            auto d = abs(a * point.x + b * point.y + c) / denominator;
            if (d <= distanceTol)
                working.emplace(j);
        }
        // Keep track of the best (i.e. biggest) set of inliners so far
        if (working.size() > best.size())
            std::swap(working, best);
        working.clear();
    }
    return best;

}


int main() {

    // Create viewer
    pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

    // Create data
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();


    // DONE: Change the max iteration and distance tolerance arguments for Ransac function
    std::unordered_set<int> inliers = Ransac(cloud, 50, .5);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

    for (int index = 0; index < cloud->points.size(); index++) {
        pcl::PointXYZ point = cloud->points[index];
        if (inliers.count(index))
            cloudInliers->points.push_back(point);
        else
            cloudOutliers->points.push_back(point);
    }


    // Render 2D point cloud with inliers and outliers
    if (inliers.size()) {
        renderPointCloud(viewer, cloudInliers, "inliers", Color(0, 1, 0));
        renderPointCloud(viewer, cloudOutliers, "outliers", Color(1, 0, 0));
    } else {
        renderPointCloud(viewer, cloud, "data");
    }

    while (!viewer->wasStopped()) {
        viewer->spinOnce();
    }

}
