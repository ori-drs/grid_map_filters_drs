/*
 * BaseHeightThresholdFilter.cpp
 *
 *  Rotates the normals to match a frame
 * 
 *  Author: Matias Mattamala
 */


#include <grid_map_filters_drs/BaseHeightThresholdFilter.hpp>
#include <grid_map_core/grid_map_core.hpp>
#include <pluginlib/class_list_macros.h>
#include <string>
#include <stdexcept>
#include <Eigen/Dense>

using namespace filters;

namespace grid_map {

template<typename T>
BaseHeightThresholdFilter<T>::BaseHeightThresholdFilter()
{
}

template<typename T>
BaseHeightThresholdFilter<T>::~BaseHeightThresholdFilter()
{
}

template<typename T>
bool BaseHeightThresholdFilter<T>::configure()
{
  // Load Parameters
  // Input layer to be processed
  if (!FilterBase<T>::getParam(std::string("input_layer"), inputLayer_)) {
    ROS_ERROR("[BaseHeightThresholdFilter] did not find parameter `input_layer`.");
    return false;
  }
  ROS_DEBUG("[BaseHeightThresholdFilter] input_layer = %s.", inputLayer_.c_str());

  // Output layer to be processed
  if (!FilterBase<T>::getParam(std::string("output_layer"), outputLayer_)) {
    ROS_ERROR("[BaseHeightThresholdFilter] did not find parameter `output_layer`.");
    return false;
  }
  ROS_DEBUG("[BaseHeightThresholdFilter] output_layer = %s.", outputLayer_.c_str());

  // Target frame
  if (!FilterBase<T>::getParam(std::string("target_frame"), targetFrame_)) {
    ROS_ERROR("[BaseHeightThresholdFilter] did not find parameter 'target_frame'.");
    return false;
  }
  ROS_DEBUG("[BaseHeightThresholdFilter] target_frame = %s.", targetFrame_.c_str());

  // Threshold
  if (!FilterBase<T>::getParam(std::string("threshold"), heightThreshold_)) {
    ROS_ERROR("[BaseHeightThresholdFilter] did not find parameter 'threshold'.");
    return false;
  }
  ROS_DEBUG("[BaseHeightThresholdFilter] threshold = %f.", heightThreshold_);

  // Set to max
  if (!FilterBase<T>::getParam(std::string("set_to_upper"), setToUpper_)) {
    ROS_ERROR("[BaseHeightThresholdFilter] did not find parameter 'set_to_upper'.");
    return false;
  }
  ROS_DEBUG("[BaseHeightThresholdFilter] set_to_upper = %f.", setToUpper_);

  // Set to min
  if (!FilterBase<T>::getParam(std::string("set_to_lower"), setToLower_)) {
    ROS_ERROR("[BaseHeightThresholdFilter] did not find parameter 'set_to_lower'.");
    return false;
  }
  ROS_DEBUG("[BaseHeightThresholdFilter] set_to_lower = %f.", setToLower_);

  // Initialize TF listener
  tfListener_ = std::make_shared<tf::TransformListener>();

  return true;
}

template<typename T>
bool BaseHeightThresholdFilter<T>::update(const T& mapIn, T& mapOut)
{
  auto tic = std::chrono::high_resolution_clock::now();

  mapOut = mapIn;

  // Check if layer exists.
  if (!mapOut.exists(inputLayer_)) {
    ROS_ERROR("[BaseHeightThresholdFilter] Check your layer type! Type %s does not exist", inputLayer_.c_str());
    return false;
  }

  // Add filtered layer
  mapOut.add(outputLayer_, mapIn[inputLayer_]);

  // Get frame of elevation map
  mapFrame_ = mapOut.getFrameId();

  // Get transformation
  Eigen::Isometry3d mapToTarget = Eigen::Isometry3d::Identity();
  
  // Recover transformation
  try {
    tf::StampedTransform mapToTargetTransform;
    tfListener_->lookupTransform(targetFrame_, mapFrame_, ros::Time(0), mapToTargetTransform);
    tf::transformTFToEigen (mapToTargetTransform, mapToTarget);
  }
  catch (tf::TransformException& ex){
    ROS_ERROR("%s",ex.what());
  }

  // Apply transformation and compare height
  for (grid_map::GridMapIterator iterator(mapOut); !iterator.isPastEnd(); ++iterator) {
    if (mapOut.isValid(*iterator, inputLayer_)){
      grid_map::Position position;
      mapOut.getPosition(*iterator, position);

      // Build 3 components of point
      Eigen::Vector3d point = Eigen::Vector3d::Identity();
      point.x() = position.x();
      point.y() = position.y();
      point.z() = mapIn.at(inputLayer_, *iterator);
      
      // Apply transformation
      point = mapToTarget * point;

      // Store in output layer
      if(point.z() > heightThreshold_){
        mapOut.at(outputLayer_, *iterator) = setToUpper_;
      } else {
        mapOut.at(outputLayer_, *iterator) = setToLower_;
      }
    }
  }
  
  // Timing
  auto toc = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsedTime = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(toc - tic);
  ROS_DEBUG_STREAM("[BaseHeightThresholdFilter] Process time: " << elapsedTime.count() << " ms");

  return true;
}

} /* namespace */

PLUGINLIB_EXPORT_CLASS(grid_map::BaseHeightThresholdFilter<grid_map::GridMap>, filters::FilterBase<grid_map::GridMap>)