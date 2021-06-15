/*
 * GeodesicDistanceField2dFilter.cpp
 *
 *  Implements a Geodesic Distance layer using a binary layer. 0 means obstacles, 1 free space
 * 
 *  Author: Matias Mattamala
 */

#include <grid_map_filters_drs/GeodesicDistanceField2dFilter.hpp>

using namespace filters;

namespace grid_map {

template<typename T>
GeodesicDistanceField2dFilter<T>::GeodesicDistanceField2dFilter()
: attractorPosition_(0, 0),
  attractorStamp_(0),
  mapFrame_("not_set")
{
}

template<typename T>
GeodesicDistanceField2dFilter<T>::~GeodesicDistanceField2dFilter()
{
}

template<typename T>
bool GeodesicDistanceField2dFilter<T>::configure()
{
  // Initialize node handle
  nodeHandle_ = ros::NodeHandle("~geodesic_distance_filter");

  // Load Parameters
  // Input layer to be processed
  if (!FilterBase<T>::getParam(std::string("input_layer"), inputLayer_)) {
    ROS_ERROR("Geodesic Distance 2D filter did not find parameter `input_layer`.");
    return false;
  }
  ROS_DEBUG("Geodesic Distance 2D filter output_layer = %s.", inputLayer_.c_str());

  // Read output_layers_prefix, to define output grid map layers prefix.
  if (!filters::FilterBase<T>::getParam(std::string("output_layer"), outputLayer_)) {
    ROS_ERROR("Geodesic Distance 2D filter did not find parameter `output_layer`.");
    return false;
  }
  ROS_DEBUG("Geodesic Distance 2D filter output_layer = %s.", outputLayer_.c_str());

  // Read threshold, to define the untraversable areas
  if (!filters::FilterBase<T>::getParam(std::string("threshold"), threshold_)) {
    ROS_ERROR("Geodesic Distance 2D filter did not find parameter `threshold`.");
    return false;
  }
  ROS_DEBUG("Geodesic Distance 2D filter threshold = %f.", threshold_);

  // Option to smooth the field
  // Read flag to binarize defining traversable and non traversable areas
  if (!filters::FilterBase<T>::getParam(std::string("use_field_smoothing"), fieldSmoothing_)) {
    ROS_ERROR("Geodesic Distance 2D filter did not find parameter `use_field_smoothing`.");
    return false;
  }
  ROS_DEBUG("Geodesic Distance 2D filter use_field_smoothing = %s.", (fieldSmoothing_? "true" : "false"));

  // Smoothing radius
  if (!filters::FilterBase<T>::getParam(std::string("field_smoothing_radius"), fieldSmoothingRadius_)) {
    ROS_ERROR("Geodesic Distance 2D filter did not find parameter `field_smoothing_radius`.");
    return false;
  }
  ROS_DEBUG("Geodesic Distance 2D filter field_smoothing_radius = %f.", fieldSmoothingRadius_);

  // Attractor subscriber topic
  if (!FilterBase<T>::getParam(std::string("attractor_topic"), attractorTopic_)) {
    ROS_ERROR("Geodesic Distance 2D filter did not find parameter 'attractor_topic'.");
    return false;
  }
  ROS_DEBUG("Geodesic Distance 2D filter attractor_topic = %s.", attractorTopic_.c_str());

  // Read option to normalize gradients
  if (!filters::FilterBase<T>::getParam(std::string("normalize_gradients"), normalizeGradients_)) {
    ROS_ERROR("SDF 2D filter did not find parameter `normalize_gradients`.");
    return false;
  }
  ROS_DEBUG("SDF 2D filter normalize_gradients = %s.", (normalizeGradients_? "true" : "false"));

  // Initialize TF listener
  tfListener_ = std::make_shared<tf::TransformListener>();

  // Initialize subscriber
  attractorSubscriber_       = nodeHandle_.subscribe(std::string(attractorTopic_), 1, &GeodesicDistanceField2dFilter::attractorCallback, this);

  // Initialize output layer variables
  obstacleLayer_  = outputLayer_ + "_obstacle_space";
  freeSpaceLayer_ = outputLayer_  + "_free_space";
  gradientXLayer_ = outputLayer_ + "_gradient_x";
  gradientYLayer_ = outputLayer_ + "_gradient_y";
  gradientZLayer_ = outputLayer_ + "_gradient_z";

  return true;
}

template<typename T>
void GeodesicDistanceField2dFilter<T>::attractorCallback(const geometry_msgs::PoseStampedConstPtr& msg) {

  if(mapFrame_ == "not_set"){
    return;
  }

  // Convert attractor goal to grid map frame
  std::string goalFrame = msg->header.frame_id;

  Eigen::Isometry3d goalPose = Eigen::Isometry3d::Identity();
  tf::poseMsgToEigen(msg->pose, goalPose);

  if (goalFrame != mapFrame_){
    tf::StampedTransform goalToMapTransform;
    
    try {
      tfListener_->lookupTransform(mapFrame_, goalFrame, ros::Time(0), goalToMapTransform);

      // Convert to Isometry3d
      Eigen::Isometry3d goalToMap = Eigen::Isometry3d::Identity();
      tf::transformTFToEigen (goalToMapTransform, goalToMap);

      // Update goal
      goalPose = goalToMap * goalPose;
    }
    catch (tf::TransformException& ex){
      ROS_ERROR("%s",ex.what());
      return;
    }
  }

  // If the timestamps are the same, skip (to avoid TF_REPEATED_DATA issue)
  if(msg->header.stamp == attractorStamp_) {
    return;
  }

  // Fill attractor coordinates
  attractorPosition_.x() = goalPose.translation().x();
  attractorPosition_.y() = goalPose.translation().y();
  attractorStamp_ = msg->header.stamp;

  ROS_DEBUG_STREAM("[GeodesicDistanceField2dFilter] Attractor: [" 
                   << attractorPosition_.x() << ", " << attractorPosition_.y() << "]" );
}

template<typename T>
bool GeodesicDistanceField2dFilter<T>::update(const T& mapIn, T& mapOut) {
  auto tic = std::chrono::high_resolution_clock::now();

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

  if(mapFrame_ == "not_set"){
    // Set standard goal at the center
    attractorPosition_.x() = mapOut.getSize()(0)/2 * resolution;
    attractorPosition_.y() = mapOut.getSize()(1)/2 * resolution;
  }

  // Get frame of elevation map
  mapFrame_ = mapOut.getFrameId();

  // Convert selected layer to OpenCV image
  cv::Mat cvLayer;
  grid_map::GridMapCvConverter::toImage<float, 1>(mapOut, inputLayer_, CV_32F, cvLayer);

  // Apply threshold and compute obstacle masks
  cv::Mat cvFreeSpaceMask = cvLayer;
  cv::Mat cvObstacleSpaceMask;

  // Binarize input layer
  cv::threshold(cvLayer, cvFreeSpaceMask, threshold_, 255, cv::THRESH_BINARY);	
  cvFreeSpaceMask.convertTo(cvFreeSpaceMask, CV_8UC1);
  cv::bitwise_not(cvFreeSpaceMask, cvObstacleSpaceMask);
  cvObstacleSpaceMask.convertTo(cvObstacleSpaceMask, CV_32F);
  cvFreeSpaceMask.convertTo(cvFreeSpaceMask, CV_32F);

  // Preallocate output layer
  cv::Mat cvGeodesicDistance(cvObstacleSpaceMask.size(), cvObstacleSpaceMask.type(), cv::Scalar(0.0));
  cv::Mat maskedGradientsX(cvObstacleSpaceMask.size(), cvObstacleSpaceMask.type(), cv::Scalar(0.0));;
  cv::Mat maskedGradientsY(cvObstacleSpaceMask.size(), cvObstacleSpaceMask.type(), cv::Scalar(0.0));;
  cv::Mat cvGradientsX(cvObstacleSpaceMask.size(), cvObstacleSpaceMask.type(), cv::Scalar(0.0));;  
  cv::Mat cvGradientsY(cvObstacleSpaceMask.size(), cvObstacleSpaceMask.type(), cv::Scalar(0.0));;
  cv::Mat cvGradientsZ(cvObstacleSpaceMask.size(), cvObstacleSpaceMask.type(), cv::Scalar(0.0));

  // Add free and occupied space as layers
  addMatAsLayer(cvObstacleSpaceMask/255, obstacleLayer_, mapOut);
  addMatAsLayer(cvFreeSpaceMask/255, freeSpaceLayer_, mapOut);

  // This little hack makes the fast marching method work
  cvObstacleSpaceMask*=10; // This is to enforce the difference between obstacles and free space
  cvObstacleSpaceMask+=1;    // This adds a baseline layer to start the propagation

  // Smooth field by applying Gaussian Smoothing
  // This is similar to the 'saturation' method used in 
  // FM2 by Javier V. Gomez: https://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=6582543
  if(fieldSmoothing_){
    int radiusInPixels =  std::max((int)std::ceil(fieldSmoothingRadius_ / mapIn.getResolution()), 3); // Minimum kernel of size 3
    radiusInPixels = (radiusInPixels % 2 == 0)? radiusInPixels + 1 : radiusInPixels;

    cv::GaussianBlur(cvObstacleSpaceMask, cvObstacleSpaceMask, cv::Size(radiusInPixels, radiusInPixels), 0);
  }

  // Prepare seeds
  std::vector<cv::Point> seeds;
  grid_map::Index index = getAttractorIndex(mapOut, attractorPosition_);

  // Compute Geodesic field
  seeds.push_back(cv::Point(index.y(), index.x())); // We need to flip the components because it's an image

  // Compute Geodesic Distance
  GeodesicDistance<float>(cvObstacleSpaceMask, seeds, cvGeodesicDistance);

  // Blur before computing gradients to smooth the image
  cv::GaussianBlur(cvGeodesicDistance, cvGeodesicDistance, cv::Size(5, 5), 0);

  // Compute gradients
  cv::Sobel(cvGeodesicDistance, cvGradientsX, -1, 0, 1, 3, resolution);
  cv::Sobel(cvGeodesicDistance, cvGradientsY, -1, 1, 0, 3, resolution);

  if(normalizeGradients_) {
    // Compute norm
    cv::Mat sqGradientsX;
    cv::Mat sqGradientsY;
    cv::pow(cvGradientsX, 2, sqGradientsX);
    cv::pow(cvGradientsY, 2, sqGradientsY);
    cv::Mat normNormal;
    cv::sqrt(sqGradientsX + sqGradientsY, normNormal);
    
    // Normalize
    cvGradientsX /= normNormal;
    cvGradientsY /= normNormal;
  }

  // Add layers
  addMatAsLayer(cvGeodesicDistance, outputLayer_, mapOut, resolution);
  addMatAsLayer(cvGradientsX, gradientXLayer_, mapOut);
  addMatAsLayer(cvGradientsY, gradientYLayer_, mapOut);
  addMatAsLayer(cvGradientsZ, gradientZLayer_, mapOut);
  
  mapOut.setBasicLayers({});

  // Timing
  auto toc = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsedTime = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(toc - tic);
  ROS_DEBUG_STREAM("[GeodesicDistanceField2dFilter] Process time: " << elapsedTime.count() << " ms");
  
  return true;
}

template<typename T>
grid_map::Index GeodesicDistanceField2dFilter<T>::getAttractorIndex(const T& gridMap, const grid_map::Position& attractorPosition) {
  // Preallocate output index
  grid_map::Index index;
  
  // Get closest attractor position
  Position closestAttractor = gridMap.getClosestPositionInMap(attractorPosition);
  
  // Check if the attractor is in traversable areas
  gridMap.getIndex(closestAttractor, index);

  // Sanity check
  index.x() = std::min(index.x(), gridMap.getSize().x()-1);
  index.y() = std::min(index.y(), gridMap.getSize().y()-1);
  index.x() = std::max(index.x(), 0);
  index.y() = std::max(index.y(), 0);

  // ROS_WARN_STREAM("attractor closest index: " << index(0) << ", " << index(1));
  bool traversable = gridMap.at(freeSpaceLayer_, index) > 0;
  // ROS_WARN_STREAM("attractor traversable? " << traversable);

  // If not traversable, we need to find a new candidate attractor
  if(!traversable) {
    double radius = gridMap.getSize()(0) * gridMap.getResolution(); // meters

    for (grid_map::SpiralIterator iterator(gridMap, closestAttractor, radius); !iterator.isPastEnd(); ++iterator) {
      if(gridMap.isValid(*iterator, freeSpaceLayer_)) {
        grid_map::Index closestIndex = *iterator;
        if(gridMap.at(freeSpaceLayer_, closestIndex) > 0) {
          index = closestIndex;
          ROS_WARN_STREAM("attractor spiral index: " << index(0) << ", " << index(1));
          break;
        }
      }
    }
  }
  ROS_WARN_STREAM("attractor final index: " << index(0) << ", " << index(1));
  // cv::waitKey(10);
  
  return index;
}

template<typename T>
void GeodesicDistanceField2dFilter<T>::addMatAsLayer(const cv::Mat& mat, const std::string& layerName, grid_map::GridMap& gridMap, double resolution)
{
  // ROS_WARN_STREAM("trying to add layer [" << layerName << "]" << mat.rows << " x " << mat.cols);
  // Get max and min
  double minDistance, maxDistance;
  cv::minMaxLoc(mat, &minDistance, &maxDistance);

  minDistance*= resolution;
  maxDistance*= resolution;

  // cv::namedWindow(layerName + "_original", cv::WINDOW_NORMAL );
  // cv::imshow(layerName + "_original", mat);
  
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
template class grid_map::GeodesicDistanceField2dFilter<grid_map::GridMap>;
// Export the filter.
PLUGINLIB_EXPORT_CLASS(grid_map::GeodesicDistanceField2dFilter<grid_map::GridMap>, filters::FilterBase<grid_map::GridMap>)