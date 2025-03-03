//----------------------------------------
// This file is part of grid_map_filters_drs
//
// Copyright (C) 2020-2025 Matías Mattamala, University of Oxford.
//
// grid_map_filters_drs is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
// License as published by the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// grid_map_filters_drs is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
// the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License along with grid_map_filters_drs.
// If not, see <http://www.gnu.org/licenses/>.
//----------------------------------------
/*
 * ChangeNormalsFrameFilter.cpp
 *
 *  Rotates the normals to match a frame
 *
 *  Author: Matias Mattamala
 */

#include <pluginlib/class_list_macros.h>
#include <Eigen/Dense>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_filters_drs/ChangeNormalsFrameFilter.hpp>
#include <stdexcept>
#include <string>

using namespace filters;

namespace grid_map {

template <typename T>
ChangeNormalsFrameFilter<T>::ChangeNormalsFrameFilter() {}

template <typename T>
ChangeNormalsFrameFilter<T>::~ChangeNormalsFrameFilter() {}

template <typename T>
bool ChangeNormalsFrameFilter<T>::configure() {
  // Setup profiler
  profiler_ptr_ = std::make_shared<Profiler>("ChangeNormalsFrameFilter");

  // Load Parameters
  // Input layer to be processed
  if (!FilterBase<T>::getParam(std::string("input_layers_prefix"), inputNormalLayersPrefix_)) {
    ROS_ERROR("[ChangeNormalsFrameFilter] did not find parameter `input_layer`.");
    return false;
  }
  ROS_DEBUG("[ChangeNormalsFrameFilter] input_layers_prefix = %s.", inputNormalLayersPrefix_.c_str());

  // Target frame
  if (!FilterBase<T>::getParam(std::string("target_frame"), targetFrame_)) {
    ROS_ERROR("[ChangeNormalsFrameFilter] did not find parameter 'target_frame'.");
    return false;
  }

  // Initialize TF listener
  tfListener_ = std::make_shared<tf::TransformListener>();

  // Preallocate normal layer names
  xInputLayer_ = inputNormalLayersPrefix_ + "x";
  yInputLayer_ = inputNormalLayersPrefix_ + "y";
  zInputLayer_ = inputNormalLayersPrefix_ + "z";
  xOutputLayer_ = inputNormalLayersPrefix_ + "in_" + targetFrame_ + "_x";
  yOutputLayer_ = inputNormalLayersPrefix_ + "in_" + targetFrame_ + "_y";
  zOutputLayer_ = inputNormalLayersPrefix_ + "in_" + targetFrame_ + "_z";

  return true;
}

template <typename T>
bool ChangeNormalsFrameFilter<T>::update(const T& mapIn, T& mapOut) {
  profiler_ptr_->startEvent("0.update");

  mapOut = mapIn;

  // Check if layer exists.
  if (!mapOut.exists(xInputLayer_)) {
    ROS_ERROR("[ChangeNormalsFrameFilter] Check your layer type! Type %s does not exist", xInputLayer_.c_str());
    return false;
  }
  if (!mapOut.exists(yInputLayer_)) {
    ROS_ERROR("[ChangeNormalsFrameFilter] Check your layer type! Type %s does not exist", yInputLayer_.c_str());
    return false;
  }
  if (!mapOut.exists(zInputLayer_)) {
    ROS_ERROR("[ChangeNormalsFrameFilter] Check your layer type! Type %s does not exist", zInputLayer_.c_str());
    return false;
  }

  // Add filtered layer
  mapOut.add(xOutputLayer_);
  mapOut.add(yOutputLayer_);
  mapOut.add(zOutputLayer_);

  // Get frame of elevation map
  mapFrame_ = mapOut.getFrameId();

  // Get transformation
  Eigen::Isometry3d mapToTarget = Eigen::Isometry3d::Identity();

  // Recover transformation
  try {
    tf::StampedTransform mapToTargetTransform;
    tfListener_->lookupTransform(targetFrame_, mapFrame_, ros::Time(0), mapToTargetTransform);
    tf::transformTFToEigen(mapToTargetTransform, mapToTarget);
  } catch (tf::TransformException& ex) {
    ROS_ERROR("%s", ex.what());
  }

  // Apply rotation
  for (grid_map::GridMapIterator iterator(mapOut); !iterator.isPastEnd(); ++iterator) {
    if (mapOut.isValid(*iterator, xInputLayer_) && mapOut.isValid(*iterator, yInputLayer_) && mapOut.isValid(*iterator, zInputLayer_)) {
      // Extract 3 components of normal
      Eigen::Vector3d normal;
      normal.x() = mapOut.at(xInputLayer_, *iterator);
      normal.y() = mapOut.at(yInputLayer_, *iterator);
      normal.z() = mapOut.at(zInputLayer_, *iterator);

      // Rotate normal
      normal = mapToTarget.rotation() * normal;

      // Store in output layer
      mapOut.at(xOutputLayer_, *iterator) = normal.x();
      mapOut.at(yOutputLayer_, *iterator) = normal.y();
      mapOut.at(zOutputLayer_, *iterator) = normal.z();
    }
  }

  // Timing
  profiler_ptr_->endEvent("0.update");
  ROS_DEBUG_STREAM_THROTTLE(1, "-- Profiler report (throttled (1s)\n" << profiler_ptr_->getReport());

  return true;
}

}  // namespace grid_map

// Explicitly define the specialization for GridMap
template class grid_map::ChangeNormalsFrameFilter<grid_map::GridMap>;
// Export the filter.
PLUGINLIB_EXPORT_CLASS(grid_map::ChangeNormalsFrameFilter<grid_map::GridMap>, filters::FilterBase<grid_map::GridMap>)