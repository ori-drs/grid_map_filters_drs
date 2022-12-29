/*
 * NanMaskFilter.cpp
 *
 *  Replaces NaNs by a fixed value
 * 
 *  Author: Matias Mattamala
 */



#include <grid_map_filters_drs/NanMaskFilter.hpp>
#include <grid_map_core/grid_map_core.hpp>
#include <pluginlib/class_list_macros.h>
#include <string>
#include <stdexcept>

using namespace filters;

namespace grid_map {

template<typename T>
NanMaskFilter<T>::NanMaskFilter()
    : setTo_("0.0")
{
}

template<typename T>
NanMaskFilter<T>::~NanMaskFilter()
{
}

template<typename T>
bool NanMaskFilter<T>::configure()
{
  // Setup profiler
  profiler_ptr_ = std::make_shared<Profiler>("NanMaskFilter");

  // Load Parameters
  // Input layer to be processed
  if (!FilterBase<T>::getParam(std::string("input_layer"), inputLayer_)) {
    ROS_ERROR("[NanMaskFilter] did not find parameter `input_layer`.");
    return false;
  }
  ROS_DEBUG("[NanMaskFilter] input_layer = %s.", inputLayer_.c_str());

  // Output layer to be processed
  if (!FilterBase<T>::getParam(std::string("output_layer"), outputLayer_)) {
    ROS_ERROR("[NanMaskFilter] did not find parameter `output_layer`.");
    return false;
  }
  ROS_DEBUG("[NanMaskFilter] output_layer = %s.", outputLayer_.c_str());

  return true;
}

template<typename T>
bool NanMaskFilter<T>::update(const T& mapIn, T& mapOut)
{
  profiler_ptr_->startEvent("0.update");
  
  mapOut = mapIn;

  // Check if layer exists.
  if (!mapOut.exists(inputLayer_)) {
    ROS_ERROR("[NanMaskFilter] Check your layer type! Type %s does not exist", inputLayer_.c_str());
    return false;
  }

  // Add filtered layer
  mapOut.add(outputLayer_, mapIn[inputLayer_]);

  for (grid_map::GridMapIterator iterator(mapOut); !iterator.isPastEnd(); ++iterator) {
    if (mapOut.isValid(*iterator, inputLayer_))
      mapOut.at(outputLayer_, *iterator) = 0.0;
    else
      mapOut.at(outputLayer_, *iterator) = 1.0;
  }

  mapOut.setBasicLayers({});

  profiler_ptr_->endEvent("0.update");
  ROS_DEBUG_STREAM_THROTTLE(1, "-- Profiler report (throttled (1s)\n" << profiler_ptr_->getReport());

  return true;
}

} /* namespace */

// Explicitly define the specialization for GridMap
template class grid_map::NanMaskFilter<grid_map::GridMap>;
// Export the filter.
PLUGINLIB_EXPORT_CLASS(grid_map::NanMaskFilter<grid_map::GridMap>, filters::FilterBase<grid_map::GridMap>)