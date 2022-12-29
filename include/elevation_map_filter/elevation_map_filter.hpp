/*
 *  Based on
 *  FiltersDemo.h
 *  by Peter Fankhauser (ETH Zurich, ANYbotics)
 * 
 *  Author: Matias Mattamala
 */


#pragma once

#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_filters_drs/utils/profiler.hpp>
#include <filters/filter_chain.hpp>
#include <std_srvs/Trigger.h>
#include <ros/ros.h>
#include <string>
#include <chrono>
#include <mutex>

using namespace grid_map;

/*!
 * Applies a chain of grid map filters to a topic and
 * republishes the resulting grid map.
 */
class ElevationMapFilter
{
public:
    /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   * @param success signalizes if filter is configured ok or not.
   */
    ElevationMapFilter(ros::NodeHandle &nodeHandle, bool &success);

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
    void callback(const grid_map_msgs::GridMap &message);

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
    bool serviceCallback(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response);

    //! ROS nodehandle.
    ros::NodeHandle &nodeHandle_;

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