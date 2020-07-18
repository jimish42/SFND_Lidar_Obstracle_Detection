/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

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

  // TODO:: Create lidar sensor
  Lidar *lidar = new Lidar(cars, 0);
  pcl::PointCloud<pcl::PointXYZ>::Ptr scanPoints;
  scanPoints = lidar->scan();
  // renderRays(viewer, lidar->position, scanPoints);
  // renderPointCloud(viewer, scanPoints, "lidar_scan", {0.5, 0.5, 0.5});

  ProcessPointClouds<pcl::PointXYZ> *pointProcessor = new ProcessPointClouds<pcl::PointXYZ>();

  std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud =
    pointProcessor->SegmentPlane(scanPoints, 100, 0.2);

  // renderPointCloud(viewer,segmentCloud.first,"planeCloud",Color(1,0,0));
  // renderPointCloud(viewer,segmentCloud.second,"obstCloud",Color(0,1,0));

  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters =
    pointProcessor->Clustering(segmentCloud.second, 1.0, 3, 30);
  int clusterId = 0;
  std::vector<Color> colors = {Color(1, 0, 0), Color(0, 1, 0), Color(0, 0, 1)};

  for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters) {
    std::cout << "cluster size ";
    pointProcessor->numPoints(cluster);
    renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId]);
    ++clusterId;
  }
  for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters) {
    Box box = pointProcessor->BoundingBox(cluster);
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

void cityBlock(pcl::visualization::PCLVisualizer::Ptr &viewer,
  ProcessPointClouds<pcl::PointXYZI> pointProcessor,
  const pcl::PointCloud<pcl::PointXYZI>::Ptr &inputCloud) {

  // ----------------------------------------------------
  // -----Open 3D viewer and display City Block     -----
  // ----------------------------------------------------

  // Experiment with the ? values and find what works best
  pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud;

  filterCloud = pointProcessor.FilterCloud(
    inputCloud,
    0.15f,
    Eigen::Vector4f(-20.f, -6.0f, -2.f, 1.f),
    Eigen::Vector4f(30.f, 7.0f, 5.f, 1));

  std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud =
    pointProcessor.Ransac3D(filterCloud, 50, 0.2);

  renderPointCloud(viewer, segmentCloud.first, "planeCloud", Color(0.5, 0.5, 0.5));
  // renderPointCloud(viewer,segmentCloud.second,"obstCloud",Color(0,1,0));

  Box box{};
  box.x_min = -1.5;
  box.y_min = -1.7;
  box.z_min = -1.0;
  box.x_max = 2.6;
  box.y_max = 1.7;
  box.z_max = -0.4;

  renderBox(viewer, box, 0, Color(0.8, 0.2, 0), 0.6f);

//   std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessor.Clustering(segmentCloud.second, 0.5, 20, 3000);

  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters =
    pointProcessor.euclideanCluster(segmentCloud.second, 0.3, 10, 1000);

  const bool renderClusters = true;
  const bool renderBoxes = true;

  int clusterId = 1;
  std::vector<Color> colors = {Color(1, 0, 0), Color(0, 1, 0), Color(0, 0, 1)};
  for (const auto &cluster : cloudClusters) {

    Color clusterColor = colors[clusterId % colors.size()];

    if (renderClusters) {
      std::cout << "cluster size ";
      pointProcessor.numPoints(cluster);
      renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), clusterColor);
    }

    if (renderBoxes) {
      Box box = pointProcessor.BoundingBox(cluster);
      renderBox(viewer, box, clusterId);
    }
    ++clusterId;
  }

  // renderPointCloud(viewer,filterCloud,"filterCloud");
  // renderPointCloud(viewer,inputCloud,"inputCloud");
}


int main(int argc, char **argv) {
  std::cout << "starting enviroment" << std::endl;

  pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
  CameraAngle setAngle = XY;
  initCamera(setAngle, viewer);
  // simpleHighway(viewer);
//  while (!viewer->wasStopped()) {
//    viewer->spinOnce();
//  }

  ProcessPointClouds<pcl::PointXYZI> pointProcessor = ProcessPointClouds<pcl::PointXYZI>();
  std::vector<boost::filesystem::path> stream = pointProcessor.streamPcd("../src/sensors/data/pcd/data_1");
  auto streamIterator = stream.begin();
  pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

  while (!viewer->wasStopped ())
  {
    // Clear viewer
    viewer->removeAllPointClouds();
    viewer->removeAllShapes();

    // Load pcd and run obstacle detection process
    inputCloudI = pointProcessor.loadPcd((*streamIterator).string());
    cityBlock(viewer, pointProcessor, inputCloudI);

    streamIterator++;
    if(streamIterator == stream.end())
      streamIterator = stream.begin();

    viewer->spinOnce ();
  }

}