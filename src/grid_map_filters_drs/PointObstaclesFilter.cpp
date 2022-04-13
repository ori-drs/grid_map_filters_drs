/*
 * PointObstaclesFilter.cpp
 *
 *  Implements a 2D SDF layer using a binary layer. 0 means obstacles, 1 free space
 * 
 *  Author: Matias Mattamala
 */


#include <grid_map_filters_drs/PointObstaclesFilter.hpp>
#include <time.h>

using namespace filters;

namespace grid_map {

template<typename T>
PointObstaclesFilter<T>::PointObstaclesFilter()
{
}

template<typename T>
PointObstaclesFilter<T>::~PointObstaclesFilter()
{
}

template<typename T>
bool PointObstaclesFilter<T>::configure()
{
  // Setup profiler
  profiler_ptr_ = std::make_shared<Profiler>("PointObstaclesFilter");

  // Initialize node handle
  nodeHandle_ = ros::NodeHandle("~point_obstacles_filter");

  // Load Parameters
  // Input layer to be processed
  if (!FilterBase<T>::getParam(std::string("input_layer"), inputLayer_)) {
    ROS_ERROR("PointObstacles Filter did not find parameter `input_layer`.");
    return false;
  }
  ROS_DEBUG("PointObstacles Filter output_layer = %s.", inputLayer_.c_str());

  // Read output_layers_prefix, to define output grid map layers prefix.
  if (!filters::FilterBase<T>::getParam(std::string("output_layer"), outputLayer_)) {
    ROS_ERROR("PointObstacles Filter did not find parameter `output_layer`.");
    return false;
  }
  ROS_DEBUG("PointObstacles Filter output_layer = %s.", outputLayer_.c_str());

  // Read threshold, to define the untraversable areas
  if (!filters::FilterBase<T>::getParam(std::string("threshold"), threshold_)) {
    ROS_ERROR("PointObstacles Filter did not find parameter `threshold`.");
    return false;
  }
  ROS_DEBUG("PointObstacles Filter threshold = %f.", threshold_);

  // Read topic to publish obstacles
  if (!filters::FilterBase<T>::getParam(std::string("obstacles_topic"), obstaclesTopic_)) {
    ROS_ERROR("PointObstacles Filter did not find parameter `obstacles_topic`.");
    return false;
  }
  ROS_DEBUG("PointObstacles Filter obstacles_topic = %s.", obstaclesTopic_);

  // Read frame
  if (!filters::FilterBase<T>::getParam(std::string("obstacles_frame"), obstaclesFrame_)) {
    ROS_ERROR("PointObstacles Filter did not find parameter `obstacles_frame`.");
    return false;
  }
  ROS_DEBUG("PointObstacles Filter obstacles_frame = %s.", obstaclesFrame_);
  
  // Initialize TF listener
  tfListener_ = std::make_shared<tf::TransformListener>();

  // Initialize subscriber
  obstaclesPublisher_ = nodeHandle_.advertise<costmap_converter::ObstacleArrayMsg>(obstaclesTopic_, 1);

  return true;
}

template<typename T>
grid_map::Position PointObstaclesFilter<T>::transformPosition(grid_map::Position& positionInOrigin,
                                                              const std::string& originFrame,
                                                              const std::string&  targetFrame) {

  grid_map::Position positionInTarget = positionInOrigin;
  Eigen::Isometry3d T_target_origin = Eigen::Isometry3d::Identity();

  if (originFrame != targetFrame){
    tf::StampedTransform tfT_target_origin;
    
    try {
      tfListener_->lookupTransform(targetFrame, originFrame, ros::Time(0), tfT_target_origin);

      // Convert to Isometry3d
      tf::transformTFToEigen (tfT_target_origin, T_target_origin);

      // Create homogeneous vector from 2d position
      Eigen::Vector3d homP_origin(positionInOrigin(0), positionInOrigin(1), 0.0);

      // Transform homogeneous position
      Eigen::Vector3d homP_target = T_target_origin * homP_origin;

      // Return x and y
      positionInTarget(0) = homP_target(0);
      positionInTarget(1) = homP_target(1);

    }
    catch (tf::TransformException& ex){
      ROS_ERROR("%s",ex.what());
      return positionInTarget;
    }
  } 
  return positionInTarget;
}


template<typename T>
bool PointObstaclesFilter<T>::update(const T& mapIn, T& mapOut) {
  // auto tic = std::chrono::high_resolution_clock::now();
  profiler_ptr_->startEvent("0.update");
  // clock_t start_t = clock();

  // Copy and fix indexing
  mapOut = mapIn;
  mapOut.convertToDefaultStartIndex();

  // Check if layer exists.
  if (!mapOut.exists(inputLayer_)) {
    ROS_ERROR("Check your threshold types! Type %s does not exist", inputLayer_.c_str());
    return false;
  }

  // Get resolution
  double resolution = mapOut.getResolution();
  std::string mapFrame = mapOut.getFrameId();
  grid_map::Position mapPosition = mapOut.getPosition();

  profiler_ptr_->startEvent("1.preprocess");
  // Convert selected layer to OpenCV image
  cv::Mat cvLayer;
  const float minValue = mapOut.get(inputLayer_).minCoeffOfFinites();
  const float maxValue = mapOut.get(inputLayer_).maxCoeffOfFinites();
  grid_map::GridMapCvConverter::toImage<float, 1>(mapOut, inputLayer_, CV_32F, 
                                                  minValue, maxValue, cvLayer);  
  cv::normalize(cvLayer, cvLayer, minValue, maxValue, cv::NORM_MINMAX);
  cvLayer.convertTo(cvLayer, CV_32F);
  cvLayer*=255;

  // Apply threshold and compute obstacle masks
  cv::Mat cvObstacleSpaceMask; // This has the obstacles in white (255)
  cv::Mat cvFreeSpaceMask; // This represents obstacles as black (0)

  cv::threshold(cvLayer, cvObstacleSpaceMask, 255*threshold_, 255, cv::THRESH_BINARY);
  cvObstacleSpaceMask.convertTo(cvObstacleSpaceMask, CV_8UC1);
  cv::bitwise_not(cvObstacleSpaceMask, cvFreeSpaceMask);

  addMatAsLayer(cvObstacleSpaceMask, "obstacle_mask", mapOut, resolution);

  profiler_ptr_->endEvent("1.preprocess");

  // Find obstacles
  profiler_ptr_->startEvent("2.find_obstacles");
  
  // Find blobs
  cv::SimpleBlobDetector::Params params;
  // // Change thresholds
  // params.minThreshold = 50;
  // params.maxThreshold = 200;

  // // Filter by Area.
  // params.filterByArea = true;
  // params.minArea = 1500;

  // // Filter by Circularity
  // params.filterByCircularity = true;
  // params.minCircularity = 0.1;

  // // Filter by Convexity
  // params.filterByConvexity = true;
  // params.minConvexity = 0.87;

  // // Filter by Inertia
  // params.filterByInertia = true;
  // params.minInertiaRatio = 0.01;

  cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);
  std::vector<cv::KeyPoint> keypoints;
  detector->detect(cvObstacleSpaceMask, keypoints);

  // Iterate keypoints
  cv::Mat cvObstacleLayer = cvObstacleSpaceMask * 0; // create a black image

  // Prepare obstacles msg
  costmap_converter::ObstacleArrayMsg pointObstacles;
  pointObstacles.header.stamp = ros::Time::now();
  pointObstacles.header.frame_id = obstaclesFrame_;


  for(auto& kp : keypoints){
    const double& x = kp.pt.x;
    const double& y = kp.pt.x;
    const double& radius = kp.size;

    // Draw them on image
    cv::circle(cvObstacleLayer, kp.pt, radius, cv::Scalar(10,10,10), cv::FILLED, cv::FILLED, 0);

    // Get position in image coordinates
    grid_map::Position positionInImage(x, y);
    // ROS_WARN_STREAM("positionInImage (" << positionInImage(0) << ", " << positionInImage(1) << ")" );

    // Convert to local map coordinates
    grid_map::Position positionInLocal = positionInImage * resolution;
    // ROS_WARN_STREAM("positionInLocal (" << positionInLocal(0) << ", " << positionInLocal(1) << ")" );

    // Convert to coordinates in grid map frame
    grid_map::Position positionInMap = positionInLocal + mapPosition;
    // ROS_WARN_STREAM("positionInMap (" << positionInMap(0) << ", " << positionInMap(1) << ")" );

    // Convert to target frame
    // TODO this is redundant, it calls tf every time
    grid_map::Position positionInTarget = transformPosition(positionInMap,
                                                            mapFrame,
                                                            obstaclesFrame_);
    // ROS_WARN_STREAM("positionInTarget (" << positionInTarget(0) << ", " << positionInTarget(1) << ")" );                                                            

    // Create obstacle msg
    costmap_converter::ObstacleMsg obstacleMsg;
    obstacleMsg.header.stamp = ros::Time::now();
    obstacleMsg.header.frame_id = obstaclesFrame_;
    geometry_msgs::Point32 point;
    point.x = positionInTarget(0);
    point.y = positionInTarget(1);
    point.z = 1.0;
    obstacleMsg.polygon.points.push_back(point);
    obstacleMsg.radius = radius * resolution;

    // Add obstacle
    pointObstacles.obstacles.push_back(obstacleMsg);
  }
  profiler_ptr_->endEvent("2.find_obstacles");

  // Add layers
  profiler_ptr_->startEvent("4.add_obstacle_layer");
  addMatAsLayer(cvObstacleLayer, outputLayer_, mapOut, resolution);
  profiler_ptr_->endEvent("4.add_obstacle_layer");
  mapOut.setBasicLayers({});

  // Publish obstacles
  obstaclesPublisher_.publish(pointObstacles);

  // Timing
  // clock_t end_t = clock();
  // double total_t = ((double)(end_t - start_t)) / CLOCKS_PER_SEC;
  profiler_ptr_->endEvent("0.update");
  ROS_DEBUG_STREAM(profiler_ptr_->getReport());

  return true;
}

template<typename T>
void PointObstaclesFilter<T>::addMatAsLayer(const cv::Mat& mat, const std::string& layerName, grid_map::GridMap& gridMap, double resolution)
{
  // Get max and min
  double minDistance, maxDistance;
  cv::minMaxLoc(mat, &minDistance, &maxDistance);

  minDistance*= resolution;
  maxDistance*= resolution;
 
  // Normalize sdf to get a greyscale image
  cv::Mat normalized;
  cv::normalize(mat, normalized, 0, 1.0, cv::NORM_MINMAX);
  normalized.convertTo(normalized, CV_32F);

  // Add layer to elevation map
  grid_map::GridMapCvConverter::addLayerFromImage<float, 1>(normalized, layerName,
                                                  gridMap, minDistance, maxDistance);
}

} /* namespace */

// Explicitly define the specialization for GridMap to have the filter implementation available for testing.
template class grid_map::PointObstaclesFilter<grid_map::GridMap>;
// Export the filter.
PLUGINLIB_EXPORT_CLASS(grid_map::PointObstaclesFilter<grid_map::GridMap>, filters::FilterBase<grid_map::GridMap>)