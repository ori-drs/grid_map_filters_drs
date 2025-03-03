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
 *  Based on
 *  FiltersDemo.h
 *  by Peter Fankhauser (ETH Zurich, ANYbotics)
 *
 *  Author: Matias Mattamala
 */

#pragma once

#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <chrono>
#include <filters/filter_chain.hpp>
#include <grid_map_filters_drs/utils/profiler.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <mutex>
#include <string>

using namespace grid_map;

/*!
 * Applies a chain of grid map filters to a topic and
 * republishes the resulting grid map.
 */
class ElevationMapFilter {
 public:
  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   * @param success signalizes if filter is configured ok or not.
   */
  ElevationMapFilter(ros::NodeHandle& nodeHandle, bool& success);

  /*!
   * Destructor.
   */
  virtual ~ElevationMapFilter();

  /*!
   * Reads and verifies the ROS parameters.
   * @return true if successful.
   */
  bool readParameters();

  /*!
   * Callback method for the incoming grid map message.
   * @param message the incoming message.
   */
  void callback(const grid_map_msgs::GridMap& message);

 private:
  /*!
   * Applies the filterchain and publishes the filtered grid map
   * @param inputMap the grid map to be processed
   */
  void applyFilterChain(grid_map::GridMap& inputMap);

  /*!
   * A service callback to force an update of the filter chain
   * @param request empty request
   * @param response empty response
   */
  bool serviceCallback(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response);

  //! ROS nodehandle.
  ros::NodeHandle& nodeHandle_;

  //! Name of the input grid map topic.
  std::string inputTopic_;

  //! Name of the output grid map topic.
  std::string outputTopic_;

  //! Grid map subscriber
  ros::Subscriber subscriber_;

  //! Grid map publisher.
  ros::Publisher publisher_;

  //! Manual update via service call
  ros::ServiceServer serverForceUpdate_;

  //! Internal grid map
  GridMap inputMap_;

  //! Filter chain.
  filters::FilterChain<grid_map::GridMap> filterChain_;

  //! Filter chain parameters name.
  std::string filterChainParametersName_;

  //! Mutex
  std::mutex mutex_;

  //! Flags
  bool elevationMapAvailable_;

  //! Debug timing
  std::chrono::high_resolution_clock::time_point latencyTic_;

  //! Profiler
  std::shared_ptr<Profiler> profiler_ptr_;
};