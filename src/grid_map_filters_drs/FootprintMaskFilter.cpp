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
 * FootprintMaskFilter.cpp
 *
 *  Replaces NaNs by a fixed value
 *
 *  Author: Matias Mattamala
 */

#include <pluginlib/class_list_macros.h>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_filters_drs/FootprintMaskFilter.hpp>
#include <stdexcept>
#include <string>

using namespace filters;

namespace grid_map {

template <typename T>
FootprintMaskFilter<T>::FootprintMaskFilter() {}

template <typename T>
FootprintMaskFilter<T>::~FootprintMaskFilter() {}

template <typename T>
bool FootprintMaskFilter<T>::configure() {
  // Setup profiler
  profiler_ptr_ = std::make_shared<Profiler>("FootprintMaskFilter");

  // Output layer
  if (!FilterBase<T>::getParam(std::string("output_layer"), outputLayer_)) {
    ROS_ERROR("[FootprintMaskFilter] did not find parameter `output_layer`.");
    return false;
  }
  ROS_DEBUG("[FootprintMaskFilter] output_layer = %s.", outputLayer_.c_str());

  // Target frame
  if (!FilterBase<T>::getParam(std::string("footprint_frame"), footprintFrame_)) {
    ROS_ERROR("[FootprintMaskFilter] did not find parameter 'footprint_frame'.");
    return false;
  }
  ROS_DEBUG("[FootprintMaskFilter] footprint_frame = %s.", footprintFrame_.c_str());

  // Type of footprint
  if (!FilterBase<T>::getParam(std::string("type"), type_)) {
    ROS_ERROR("[FootprintMaskFilter] did not find parameter 'type'.");
    return false;
  }
  ROS_DEBUG("[FootprintMaskFilter] type = %s.", type_.c_str());

  // Robot length
  if (!FilterBase<T>::getParam(std::string("length"), length_)) {
    ROS_ERROR("[FootprintMaskFilter] did not find parameter `length`.");
    return false;
  }
  ROS_DEBUG("[FootprintMaskFilter] length = %f.", length_);

  // Robot width
  if (!FilterBase<T>::getParam(std::string("width"), width_)) {
    ROS_ERROR("[FootprintMaskFilter] did not find parameter `width`.");
    return false;
  }
  ROS_DEBUG("[FootprintMaskFilter] width = %f.", width_);

  // Clearance
  clearance_ = 0.0;
  if (!FilterBase<T>::getParam(std::string("width"), clearance_)) {
    ROS_ERROR("[FootprintMaskFilter] did not find parameter `width`. Using default %f", clearance_);
  }
  ROS_DEBUG("[FootprintMaskFilter] clearance = %f.", clearance_);

  // Initialize TF listener
  tfListener_ = std::make_shared<tf::TransformListener>();

  return true;
}

template <typename T>
bool FootprintMaskFilter<T>::update(const T& mapIn, T& mapOut) {
  profiler_ptr_->startEvent("0.update");
  mapOut = mapIn;

  // Get frame of elevation map
  mapFrame_ = mapOut.getFrameId();

  // Add filtered layer
  mapOut.add(outputLayer_, 0.0);

  // Get transformation
  Eigen::Isometry3d T_map_footprint = Eigen::Isometry3d::Identity();

  // Recover transformation of footprint in map frame
  try {
    tf::StampedTransform Ttf_map_footprint;
    tfListener_->lookupTransform(mapFrame_, footprintFrame_, ros::Time(0), Ttf_map_footprint);
    tf::transformTFToEigen(Ttf_map_footprint, T_map_footprint);
  } catch (tf::TransformException& ex) {
    ROS_ERROR("%s", ex.what());
  }

  // TODO: ADD orientation

  if (type_ == "rectangle") {
    grid_map::Polygon polygon;
    polygon.setFrameId(footprintFrame_);

    // Counter clock wise iteration
    std::vector<std::vector<float>> signs{{1, 1}, {-1, 1}, {-1, -1}, {1, -1}};
    for (auto s : signs) {
      Eigen::Vector3d p_footprint(s[0] * 0.5 * (length_ + clearance_), s[1] * 0.5 * (width_ + clearance_), 0.0);
      Eigen::Vector3d p_map = T_map_footprint * p_footprint;
      polygon.addVertex(Position(p_map(0), p_map(1)));
    }

    for (grid_map::PolygonIterator iterator(mapOut, polygon); !iterator.isPastEnd(); ++iterator) {
      mapOut.at(outputLayer_, *iterator) = 1.0;
    }

  } else if (type_ == "circle") {
    Position center(T_map_footprint(0, 3), T_map_footprint(1, 3));
    double radius = std::hypot(length_*0.5, width_*0.5) + clearance_;

    for (grid_map::CircleIterator iterator(mapOut, center, radius); !iterator.isPastEnd(); ++iterator) {
      mapOut.at(outputLayer_, *iterator) = 1.0;
    }
  } else {
    throw std::invalid_argument("type [" + type_ + "] is not valid");
  }

  mapOut.setBasicLayers({});

  profiler_ptr_->endEvent("0.update");
  ROS_DEBUG_STREAM_THROTTLE(1, "-- Profiler report (throttled (1s)\n" << profiler_ptr_->getReport());

  return true;
}

}  // namespace grid_map

// Explicitly define the specialization for GridMap
template class grid_map::FootprintMaskFilter<grid_map::GridMap>;
// Export the filter.
PLUGINLIB_EXPORT_CLASS(grid_map::FootprintMaskFilter<grid_map::GridMap>, filters::FilterBase<grid_map::GridMap>)