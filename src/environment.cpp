/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"
#include "processing.h"
#include <memory>
#include <pcl/impl/point_types.hpp>

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr &viewer) {

    Car egoCar(Vect3(0, 0, 0), Vect3(4, 2, 2), Color(0, 1, 0), "egoCar");
    Car car1(Vect3(15, 0, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car1");
    Car car2(Vect3(8, -4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car2");
    Car car3(Vect3(-12, 4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car3");

    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if (renderScene) {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr &viewer) {
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------

    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);

    // DONE:: Create lidar sensor
    auto pLidar = std::make_unique<Lidar>(cars, 0);

    // DONE:: Create point processor
    auto cloud = pLidar->scan();
    // renderRays(viewer, pLidar->position, cloud);
    // renderPointCloud(viewer, cloud, "cloud");

    ProcessPointClouds<pcl::PointXYZ> process;
    auto segmentedCloud = process.SegmentPlane(cloud, 100, .2);
    // renderPointCloud(viewer, segmentedCloud.first, "Obstructions", Color(1, 0, 0));
    renderPointCloud(viewer, segmentedCloud.second, "Plane", Color(0, 1, 0));

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = process.Clustering(segmentedCloud.first,
                                                                                        1.0,
                                                                                        3,
                                                                                        30);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1, 0, 0), Color(1, 1, 0), Color(0, 0, 1)};

    for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters) {
        std::cout << "cluster size ";
        process.numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId]);
        Box box = process.BoundingBox(cluster);
        renderBox(viewer, box, clusterId);
        ++clusterId;
    }
}


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr &viewer) {

    viewer->setBackgroundColor(0, 0, 0);

    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;

    switch (setAngle) {
        case XY :
            viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0);
            break;
        case TopDown :
            viewer->setCameraPosition(0, 0, distance, 1, 0, 1);
            break;
        case Side :
            viewer->setCameraPosition(0, -distance, 0, 0, 0, 1);
            break;
        case FPS :
            viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if (setAngle != FPS)
        viewer->addCoordinateSystem(1.0);
}

class Timer {
    std::chrono::time_point<std::chrono::steady_clock> startTime = std::chrono::steady_clock::now();
public:
    std::chrono::duration<double> elapsed() const {
        return std::chrono::steady_clock::now() - startTime;
    }

    void reset() {
        startTime = std::chrono::steady_clock::now();
    }

    long fetch_and_restart() {
        auto savedStart = startTime;
        reset();
        std::chrono::duration<double, std::milli> res = std::chrono::steady_clock::now() - savedStart;
        return res.count();
    }
};

void cityBlock(pcl::visualization::PCLVisualizer::Ptr &viewer) {
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ----------------------------------------------------

    ProcessPointClouds<pcl::PointXYZI> processor;
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = processor.loadPcd(
            "../src/sensors/data/pcd/data_1/0000000000.pcd");

    Timer theTimer;
    auto filteredCloud = processor.FilterCloud(inputCloud, .1, Eigen::Vector4f(-20, -10, -10, 1),
                                               Eigen::Vector4f(40, 10, 10, 1));

    std::cout << "Filtering took " << theTimer.fetch_and_restart() << " milliseconds" << std::endl;

    auto segmentedCloud = Ransac(filteredCloud, 100, .2);

    std::cout << "Segmentation took " << theTimer.fetch_and_restart() << " milliseconds" << std::endl;

    renderPointCloud(viewer, segmentedCloud.second, "Plane", Color(0, 1, 0));

    theTimer.fetch_and_restart();

    std::vector<std::vector<float>> points(segmentedCloud.first->size());
    // typedef std::vector<pcl::PointXYZI, Eigen::aligned_allocator<pcl::PointXYZI>> PointsVector;
    for (int i = 0; i < segmentedCloud.first->size(); ++i) {
        auto & source_point = segmentedCloud.first->points[i];
        std::vector<float> point_vec = {source_point.x, source_point.y, source_point.z, source_point.intensity};
        points[i] = point_vec;
    }

    auto *tree = new KdTree;

    for (int i = 0; i < points.size(); i++)
        tree->insert(points[i], i);

    std::cout << "Building the KD_tree took " << theTimer.fetch_and_restart() << " milliseconds" << std::endl;

    std::vector<std::vector<int>> clusters_vec = euclideanCluster(points, tree, .2, 50);

    std::cout << "Clustering took " << theTimer.fetch_and_restart() << " milliseconds" << std::endl;

    // typedef std::vector<PointT, Eigen::aligned_allocator<PointT> > VectorType;
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters;
    for (const auto &cluster_vec: clusters_vec) {
        typename pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZI>);
        for (const auto &index: cluster_vec) {
            pcl::PointXYZI newPoint;
            newPoint.x = points[index][0];
            newPoint.y = points[index][1];
            newPoint.z = points[index][2];
            newPoint.intensity = points[index][3];
            cloud_cluster->points.emplace_back(newPoint);
        }
        cloud_cluster->height = 1;
        cloud_cluster->width = clusters_vec.size();
        cloud_cluster->is_dense = true;
        cloudClusters.emplace_back(cloud_cluster);
    }

    std::cout << "Handling the clustering output " << theTimer.fetch_and_restart() << " milliseconds" << std::endl;

    std::cout << "Got " << cloudClusters.size() << " cluster(s)." << std::endl;
    int clusterId = 0;
    std::vector<Color> colors = {Color(1, 0, 0), Color(1, 1, 0), Color(0, 0, 1)};

    for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters) {
        std::cout << "cluster size ";
        processor.numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId % colors.size()]);
        Box box = processor.BoundingBox(cluster);
        renderBox(viewer, box, clusterId);
        ++clusterId;
    }
}


int main(int argc, char **argv) {
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    // simpleHighway(viewer);
    cityBlock(viewer);

    while (!viewer->wasStopped()) {
        viewer->spinOnce();
    }
}