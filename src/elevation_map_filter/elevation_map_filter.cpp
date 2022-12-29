/*
 *  Based on
 *  FiltersDemo.cpp
 *  by Peter Fankhauser (ETH Zurich, ANYbotics)
 * 
 *  Author: Matias Mattamala
 */


#include <elevation_map_filter/elevation_map_filter.hpp>

using namespace grid_map;

ElevationMapFilter::ElevationMapFilter(ros::NodeHandle &nodeHandle, bool &success)
    : nodeHandle_(nodeHandle),
      elevationMapAvailable_(false),
      filterChain_("grid_map::GridMap")
{
  if (!readParameters())
  {
    success = false;
    return;
  }

  // Setup subscriber
  subscriber_ = nodeHandle_.subscribe(inputTopic_, 10, &ElevationMapFilter::callback, this);
  // Setup publisher
  publisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>(outputTopic_, 100);
  // Setup service server
  serverForceUpdate_   = nodeHandle_.advertiseService("force_update", &ElevationMapFilter::serviceCallback, this);
  // Initialize latency callback
  latencyTic_ = std::chrono::high_resolution_clock::now();

  // Setup filter chain.
  if (!filterChain_.configure(filterChainParametersName_, nodeHandle))
  {
    ROS_ERROR("Could not configure the filter chain!");
    success = false;
    return;
  }

  // Setup profiler
  profiler_ptr_ = std::make_shared<Profiler>("ElevationMapFilter");


  success = true;
}

ElevationMapFilter::~ElevationMapFilter()
{
}

bool ElevationMapFilter::readParameters()
{
  if (!nodeHandle_.getParam("input_topic", inputTopic_))
  {
    ROS_ERROR("Could not read parameter `input_topic`.");
    return false;
  }
  nodeHandle_.param("output_topic", outputTopic_, std::string("output"));
  nodeHandle_.param("filter_chain_parameter_name", filterChainParametersName_, std::string("grid_map_filters"));
  return true;
}

void ElevationMapFilter::applyFilterChain(grid_map::GridMap& inputMap)
{
  grid_map::GridMap outputMap;

  // auto tic = std::chrono::high_resolution_clock::now();
  profiler_ptr_->startEvent("0.update");
  if (!filterChain_.update(inputMap, outputMap))
  {
    ROS_ERROR("[Filter chain] Could not update the grid map filter chain!");
    return;
  }

  // sleep(2);

  // Timing
  profiler_ptr_->endEvent("0.update");
  ROS_DEBUG_STREAM(profiler_ptr_->getReport());
  
  // auto toc = std::chrono::high_resolution_clock::now();
  // std::chrono::duration<double> elapsedTime = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(toc - tic);
  // ROS_DEBUG_STREAM("[Filter chain] Time required to apply filter chain: " << elapsedTime.count() << " ms");

  // Publish filtered output grid map.
  grid_map_msgs::GridMap outputMessage;
  GridMapRosConverter::toMessage(outputMap, outputMessage);
  publisher_.publish(outputMessage);
}

void ElevationMapFilter::callback(const grid_map_msgs::GridMap &message)
{
  auto tic = std::chrono::high_resolution_clock::now();

  std::chrono::duration<double> callbackTime = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(tic - latencyTic_);
  ROS_DEBUG_STREAM("[Filter chain] latency callback: " << callbackTime.count() << " ms");

  // Convert message to map
  GridMapRosConverter::fromMessage(message, inputMap_);

  // Elevation map available
  elevationMapAvailable_ = true;
  
  // Apply filter chain.
  applyFilterChain(inputMap_);

  auto toc = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsedTime = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(toc - tic);
  ROS_DEBUG_STREAM("[Filter chain] Time required for the callback: " << elapsedTime.count() << " ms");

  latencyTic_ = std::chrono::high_resolution_clock::now();
}

bool ElevationMapFilter::serviceCallback(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response)
{
  if(elevationMapAvailable_) {
    auto tic = std::chrono::high_resolution_clock::now();
    ROS_DEBUG_STREAM("[Filter chain] Triggering filter chain with service");
    applyFilterChain(inputMap_);
    auto toc = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsedTime = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(toc - tic);
    ROS_DEBUG_STREAM("[Filter chain] Time required to apply filter chain: " << elapsedTime.count() << " ms");

    // Fill response
    response.success = true;
    response.message = "done";
    return true;
  }

  return false;

}