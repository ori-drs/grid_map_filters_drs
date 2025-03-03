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
 * LearnedMotionCostsFilter.cpp
 *
 *  Request motion costs (risk, energy, time) from the gpu_path_optimizer package using service calls
 *
 *  Author: Matias Mattamala
 */

#include <grid_map_filters_drs/LearnedMotionCostsFilter.hpp>

using namespace filters;

namespace grid_map {

template <typename T>
LearnedMotionCostsFilter<T>::LearnedMotionCostsFilter() {}

template <typename T>
LearnedMotionCostsFilter<T>::~LearnedMotionCostsFilter() {}

template <typename T>
bool LearnedMotionCostsFilter<T>::configure() {
  // Setup profiler
  profiler_ptr_ = std::make_shared<Profiler>("LearnedMotionCostsFilter");

  // Initialize node handle
  nodeHandle_ = ros::NodeHandle("~learned_motion_cost_filter");

  // Load Parameters
  // Input layer to be processed
  if (!FilterBase<T>::getParam(std::string("input_layer"), inputLayer_)) {
    ROS_ERROR("Learned Motion Costs Filter filter did not find parameter `input_layer`.");
    return false;
  }
  ROS_DEBUG("Learned Motion Costs Filter filter output_layer = %s.", inputLayer_.c_str());

  // Read output_layers_prefix, to define output grid map layers prefix.
  if (!filters::FilterBase<T>::getParam(std::string("output_layer"), outputLayer_)) {
    ROS_ERROR("Learned Motion Costs Filter filter did not find parameter `output_layer`.");
    return false;
  }
  ROS_DEBUG("Learned Motion Costs Filter filter output_layer = %s.", outputLayer_.c_str());

  // Motion costs layer to be used.
  if (!filters::FilterBase<T>::getParam(std::string("cost_layer"), costLayer_)) {
    ROS_ERROR("Learned Motion Costs Filter filter did not find parameter `cost_layer`.");
    return false;
  }
  ROS_DEBUG("Learned Motion Costs Filter filter cost_layer = %s.", costLayer_.c_str());

  // Server service name topic
  if (!FilterBase<T>::getParam(std::string("service_name"), serviceName_)) {
    ROS_ERROR("Learned Motion Costs Filter filter did not find parameter 'service_name'.");
    return false;
  }
  ROS_DEBUG("Learned Motion Costs Filter filter service_name = %s.", serviceName_.c_str());

  // Initialize TF listener
  tfListener_ = std::make_shared<tf::TransformListener>();

  // Initialize subscriber
  serviceClient_ = nodeHandle_.serviceClient<gpu_path_optimizer::ComputeTraversability>(serviceName_);

  return true;
}

template <typename T>
bool LearnedMotionCostsFilter<T>::update(const T& mapIn, T& mapOut) {
  profiler_ptr_->startEvent("0.update");

  // Copy and fix indexing
  mapOut = mapIn;
  mapOut.convertToDefaultStartIndex();

  // Check if layer exists.
  if (!mapOut.exists(inputLayer_)) {
    ROS_ERROR("Check your threshold types! Type %s does not exist", inputLayer_.c_str());
    return false;
  }

  // Get temporal grid_map
  grid_map::GridMap requestMap = mapIn;
  for (const auto& layer : mapIn.getLayers()) {
    if (layer == inputLayer_) {
      continue;
    }
    if (!requestMap.erase(layer)) {
      ROS_ERROR("Could not remove type %s.", layer.c_str());
    }
  }

  profiler_ptr_->startEvent("1.service_call");
  // Convert to ROS msg
  grid_map_msgs::GridMap requestMessage;
  GridMapRosConverter::toMessage(requestMap, requestMessage);

  // Prepare request
  gpu_path_optimizer::ComputeTraversability service;
  service.request.input = requestMessage;

  // Call service
  if (!serviceClient_.call(service)) {
    ROS_ERROR_STREAM("Failed to call service [" << serviceName_ << "]");
    return false;
  }
  profiler_ptr_->endEvent("1.service_call");

  ROS_INFO("Requested cost to gpu_path_optimizer server");

  // Extract processed layer
  grid_map::GridMap responseMap;
  GridMapRosConverter::fromMessage(service.response.processed, responseMap);

  // Add layer
  mapOut.add(outputLayer_, responseMap.get(costLayer_));
  mapOut.setBasicLayers({});

  // Timing
  profiler_ptr_->endEvent("0.update");
  ROS_DEBUG_STREAM_THROTTLE(1, "-- Profiler report (throttled (1s)\n" << profiler_ptr_->getReport());

  return true;
}

}  // namespace grid_map

// Explicitly define the specialization for GridMap to have the filter implementation available for testing.
template class grid_map::LearnedMotionCostsFilter<grid_map::GridMap>;
// Export the filter.
PLUGINLIB_EXPORT_CLASS(grid_map::LearnedMotionCostsFilter<grid_map::GridMap>, filters::FilterBase<grid_map::GridMap>)