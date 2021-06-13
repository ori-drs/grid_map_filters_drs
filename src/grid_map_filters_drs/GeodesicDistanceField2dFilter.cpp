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
  goalStamp_(0),
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

  // Read flag to binarize definig traversable and non traversable areas
  if (!filters::FilterBase<T>::getParam(std::string("use_binarization"), useBinarization_)) {
    ROS_ERROR("Geodesic Distance 2D filter did not find parameter `use_binarization`.");
    return false;
  }
  ROS_DEBUG("Geodesic Distance 2D filter use_binarization = %s.", (useBinarization_? "true" : "false"));

  // Attractor subscriber topic
  if (!FilterBase<T>::getParam(std::string("attractor_topic"), attractorTopic_)) {
    ROS_ERROR("Geodesic Distance 2D filter did not find parameter 'attractor_topic'.");
    return false;
  }
  ROS_DEBUG("Geodesic Distance 2D filter attractor_topic = %s.", attractorTopic_.c_str());

  // Force update of filter chain service
  if (!FilterBase<T>::getParam(std::string("filter_chain_update_service"), filterChainUpdateService_)) {
    ROS_ERROR("Geodesic Distance 2D filter did not find parameter 'filter_chain_force_update_service'.");
    return false;
  }
  ROS_DEBUG("Geodesic Distance 2D filter filter_chain_update_service = %s.", filterChainUpdateService_.c_str());

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
  // attractorSubscriberRviz_   = nodeHandle_.subscribe(std::string("/goal"), 1, &GeodesicDistanceField2dFilter::attractorCallback, this);
  // attractorSubscriberRviz2_  = nodeHandle_.subscribe(std::string("/move_base_simple/goal"), 1, &GeodesicDistanceField2dFilter::attractorCallback, this);

  // Initialize 
  forceUpdateClient_ = nodeHandle_.serviceClient<std_srvs::Trigger>(filterChainUpdateService_);

  return true;
}

template<typename T>
void GeodesicDistanceField2dFilter<T>::attractorCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
  // ROS_DEBUG("[GeodesicDistanceField2dFilter] Update attractor position");
  // ROS_DEBUG_STREAM("[GeodesicDistanceField2dFilter] Map frame: " << mapFrame_);

  if(mapFrame_ == "not_set"){
    return;
  }
  std::string goalFrame = msg->header.frame_id;

  Eigen::Isometry3d goalPose = Eigen::Isometry3d::Identity();
  tf::poseMsgToEigen(msg->pose, goalPose);
  // ROS_DEBUG_STREAM("[GeodesicDistanceField2dFilter] TF Goal pose \n" << msg->pose);
  // ROS_DEBUG_STREAM("[GeodesicDistanceField2dFilter] Goal pose \n" << goalPose.matrix());

  if (goalFrame != mapFrame_){
    tf::StampedTransform goalToMapTransform;
    
    try {
      // ROS_DEBUG_STREAM("[GeodesicDistanceField2dFilter] Requesting transformation from " << goalFrame << " to " << mapFrame_);
      tfListener_->lookupTransform(mapFrame_, goalFrame, ros::Time(0), goalToMapTransform);

      // Convert to Isometry3d
      Eigen::Isometry3d goalToMap = Eigen::Isometry3d::Identity();
      tf::transformTFToEigen (goalToMapTransform, goalToMap);

      // ROS_DEBUG_STREAM("[GeodesicDistanceField2dFilter] Goal to map pose \n" << goalToMap.matrix());

      
      // Update goal
      goalPose = goalToMap * goalPose;
      // ROS_DEBUG_STREAM("[GeodesicDistanceField2dFilter] Goal in map \n" << goalPose.matrix());
    }
    catch (tf::TransformException& ex){
      ROS_ERROR("%s",ex.what());
      return;
    }
  }

  if(msg->header.stamp == goalStamp_) {
    // ROS_DEBUG_STREAM("[GeodesicDistanceField2dFilter] Goal has the same stamp: [old]" << goalStamp_ << ", [new] " << msg->header.stamp );
    return;
  }

  // // Fill attractor coordinates
  // if(std::fabs(goalPose.translation().x() - attractorPosition_.x()) < 1e-2 && 
  //     std::fabs(goalPose.translation().y() - attractorPosition_.y()) < 1e-2) {
  //     ROS_DEBUG_STREAM("[GeodesicDistanceField2dFilter] Same goal, do not update \n Goal:      " << goalPose.translation().transpose() 
  //                       << "\n Attractor: " << attractorPosition_.transpose()); 
  //     return;
  //   }
  
  // ROS_DEBUG_STREAM("[GeodesicDistanceField2dFilter] New goal, need to update  \n Goal:      " << goalPose.translation().transpose() 
  //                       << "\n Attractor: " << attractorPosition_.transpose());
  attractorPosition_.x() = goalPose.translation().x();
  attractorPosition_.y() = goalPose.translation().y();
  goalStamp_ = msg->header.stamp;

  // Trigger filter chain
  // ROS_DEBUG_STREAM("[GeodesicDistanceField2dFilter] Triggering filter chain at " << forceUpdateClient_.getService());

  std_srvs::Trigger srv;

  // TODO: Inspect this in simulation
  // forceUpdateClient_.call(srv);
  // if (forceUpdateClient_.call(srv))   {
  //   ROS_WARN_STREAM("Response: " << std::string(srv.response.message));
  // }
  // else {
  //   ROS_ERROR_STREAM("Failed to call service " << filterChainUpdateService_);
  // }
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
  cv::Mat cvObstacleSpace = cvLayer;
  cv::Mat cvFreeSpace;
  cv::Mat cvFreeSpaceFloat;

  if(useBinarization_){
    cv::threshold(cvLayer, cvObstacleSpace, threshold_, 255, cv::THRESH_BINARY);	
    cvObstacleSpace.convertTo(cvObstacleSpace, CV_8UC1);
    cv::bitwise_not(cvObstacleSpace, cvFreeSpace);
  }
  
  cvFreeSpace.convertTo(cvFreeSpaceFloat, CV_32F);

  // This little hack makes the fast marching method work
  cvFreeSpaceFloat*=10; // This is to enforce the difference between obstacles and free space
  cvFreeSpaceFloat+=1;    // This adds a baseline layer to start the propagation
  // cv::GaussianBlur(cvFreeSpaceFloat, cvFreeSpaceFloat, cv::Size(3, 3), 0);

  // Blur before computing gradients to smooth the image
  // int blurSize = 11;
  // cv::GaussianBlur(cvFreeSpaceFloat, cvFreeSpaceFloat, cv::Size(blurSize, blurSize), 0);

  // Preallocate output layer
  cv::Mat cvGeodesicDistance(cvFreeSpaceFloat.size(), cvFreeSpaceFloat.type(), cv::Scalar(0.0));
  cv::Mat maskedGradientsX = cvGeodesicDistance.clone();
  cv::Mat maskedGradientsY = cvGeodesicDistance.clone();
  cv::Mat cvGradientsX = cvGeodesicDistance.clone();
  cv::Mat cvGradientsY = cvGeodesicDistance.clone();
  cv::Mat cvGradientsZ = cvGeodesicDistance.clone();

  // Prepare seeds
  std::vector<cv::Point> seeds;
  grid_map::Index index; 
  bool inMap = mapOut.getIndex(attractorPosition_, index);

  // If the seed (attractor) is within the map, compute everything  
  if(inMap){
    seeds.push_back(cv::Point(index.y(), index.x())); // We need to flip the components because it's an image

    // Compute Geodesic Distance
    GeodesicDistance<float>(cvFreeSpaceFloat, seeds, cvGeodesicDistance);

    // Blur before computing gradients to smooth the image
    cv::GaussianBlur(cvGeodesicDistance, cvGeodesicDistance, cv::Size(5, 5), 0);

    // Compute gradients
    cv::Sobel(cvGeodesicDistance, cvGradientsX, -1, 0, 1, 3, resolution);
    cv::Sobel(cvGeodesicDistance, cvGradientsY, -1, 1, 0, 3, resolution);
    // cvGradientsX.copyTo(cvGradientsX, cvFreeSpace);
    // cvGradientsY.copyTo(cvGradientsY, cvFreeSpace);

    if(normalizeGradients_) {
      // Normalize gradients
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

    // Mask
    // cv::Mat maskGradX;
    // cv::inRange(cvGradientsX, cv::Scalar(-10.0), cv::Scalar(10.0), maskGradX);
    // cvGradientsX.copyTo(maskedGradientsX, maskGradX);
    // cv::Mat maskGradY;
    // cv::inRange(cvGradientsY, cv::Scalar(-10.0), cv::Scalar(10.0), maskGradY);
    // cvGradientsY.copyTo(maskedGradientsY, maskGradY);
  } else {
    ROS_WARN_STREAM("[GeodesicDistanceField2dFilter] Attractor: [" << attractorPosition_.x() << ", " << attractorPosition_.y() << "] not within grid map." );
  }

  // Add layers
  addMatAsLayer(cvGeodesicDistance, outputLayer_, mapOut, resolution);
  addMatAsLayer(cvObstacleSpace, outputLayer_+"_obstacle_space", mapOut);
  addMatAsLayer(cvFreeSpace, outputLayer_+"_free_space", mapOut);
  addMatAsLayer(cvGradientsX, outputLayer_+"_gradient_x", mapOut);
  addMatAsLayer(cvGradientsY, outputLayer_+"_gradient_y", mapOut);
  addMatAsLayer(cvGradientsZ, outputLayer_+"_gradient_z", mapOut);
  
  mapOut.setBasicLayers({});

  auto toc = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsedTime = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(toc - tic);
  ROS_DEBUG_STREAM("[GeodesicDistanceField2dFilter] Process time: " << elapsedTime.count() << " ms");
  
  return true;
}

template<typename T>
void GeodesicDistanceField2dFilter<T>::addMatAsLayer(const cv::Mat& mat, const std::string& layerName, grid_map::GridMap& gridMap, double resolution)
{
  // Get max and min
  double minDistance, maxDistance;
  cv::minMaxLoc(mat, &minDistance, &maxDistance);
  // ROS_WARN_STREAM("[" << layerName << "] mindist: " << minDistance << ", maxdist: " << maxDistance);

  minDistance*= resolution;
  maxDistance*= resolution;

  // cv::namedWindow(layerName + "_original", cv::WINDOW_NORMAL );
  // cv::imshow(layerName + "_original", mat);
  
  // Normalize sdf to get a greyscale image
  cv::Mat normalized;
  cv::normalize(mat, normalized, 0, 1.0, cv::NORM_MINMAX);
  normalized.convertTo(normalized, CV_32F);

  // cv::namedWindow(layerName + "_normalized", cv::WINDOW_NORMAL );
  // cv::imshow(layerName + "_normalized", normalized);

  // Add SDF layer to elevation map
  grid_map::GridMapCvConverter::addLayerFromImage<float, 1>(normalized, layerName,
                                                  gridMap, minDistance, maxDistance);
}

} /* namespace */

// Explicitly define the specialization for GridMap to have the filter implementation available for testing.
template class grid_map::GeodesicDistanceField2dFilter<grid_map::GridMap>;
// Export the filter.
PLUGINLIB_EXPORT_CLASS(grid_map::GeodesicDistanceField2dFilter<grid_map::GridMap>, filters::FilterBase<grid_map::GridMap>)