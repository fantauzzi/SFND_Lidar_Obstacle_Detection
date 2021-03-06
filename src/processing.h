#pragma once

#include "processing.h"
#include "kdtree.h"
#include <algorithm>
#include <chrono>
#include <unordered_set>
#include <pcl/common/projection_matrix.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

std::vector<std::vector<int>>
euclideanCluster(const std::vector<std::vector<float>> &points, KdTree *tree, float distanceTol, int minSize);

std::pair<typename pcl::PointCloud<pcl::PointXYZI>::Ptr, typename pcl::PointCloud<pcl::PointXYZI>::Ptr>
Ransac(const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud, int maxIterations, float distanceTol);

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>
separateClouds(pcl::PointIndices::Ptr inliers,
               typename pcl::PointCloud<PointT>::Ptr cloud) {
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    typename pcl::PointCloud<PointT>::Ptr plane_cloud_p(new pcl::PointCloud<PointT>);
    extract.filter(*plane_cloud_p);
    extract.setNegative(true);
    typename pcl::PointCloud<PointT>::Ptr obstacles_cloud_p(new pcl::PointCloud<PointT>);
    extract.filter(*obstacles_cloud_p);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacles_cloud_p, plane_cloud_p);
    assert(inliers->indices.size()==segResult.second->size());
    assert(segResult.second->size()+segResult.first->size()==cloud->size());
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>
ransacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations,
            float distanceThreshold) {
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    // pcl::PointIndices::Ptr inliers;
    // DONE:: Fill in this function to find inliers for the cloud.

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    // Create the segmentation object
    pcl::SACSegmentation <PointT> seg;
    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.empty())
        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = separateClouds<PointT>(
            inliers, cloud);
    return segResult;
}
