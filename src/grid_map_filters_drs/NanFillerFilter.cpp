/*
 * NanFillerFilter.cpp
 *
 *  Replaces NaNs by a fixed value
 *
 *  Author: Matias Mattamala
 */

#include <pluginlib/class_list_macros.h>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_filters_drs/NanFillerFilter.hpp>
#include <stdexcept>
#include <string>

using namespace filters;

namespace grid_map {

template <typename T>
NanFillerFilter<T>::NanFillerFilter() : setTo_("0.0") {}

template <typename T>
NanFillerFilter<T>::~NanFillerFilter() {}

template <typename T>
bool NanFillerFilter<T>::configure() {
  // Setup profiler
  profiler_ptr_ = std::make_shared<Profiler>("NanFillerFilter");

  // Load Parameters
  // Input layer to be processed
  if (!FilterBase<T>::getParam(std::string("input_layer"), inputLayer_)) {
    ROS_ERROR("[NanFillerFilter] did not find parameter `input_layer`.");
    return false;
  }
  ROS_DEBUG("[NanFillerFilter] input_layer = %s.", inputLayer_.c_str());

  // Output layer to be processed
  if (!FilterBase<T>::getParam(std::string("output_layer"), outputLayer_)) {
    ROS_ERROR("[NanFillerFilter] did not find parameter `output_layer`.");
    return false;
  }
  ROS_DEBUG("[NanFillerFilter] output_layer = %s.", outputLayer_.c_str());

  if (!FilterBase<T>::getParam(std::string("set_to"), setTo_)) {
    ROS_ERROR("[NanFillerFilter] not find parameter 'set_to'.");
    return false;
  }
  ROS_DEBUG("[NanFillerFilter] set_to = %s.", setTo_.c_str());

  if (!FilterBase<T>::getParam(std::string("value"), value_)) {
    ROS_ERROR("[NanFillerFilter] did not find parameter 'set_to'.");
    return false;
  }
  ROS_DEBUG("[NanFillerFilter] value = %f.", value_);

  return true;
}

template <typename T>
bool NanFillerFilter<T>::update(const T& mapIn, T& mapOut) {
  profiler_ptr_->startEvent("0.update");

  mapOut = mapIn;

  // Check if layer exists.
  if (!mapOut.exists(inputLayer_)) {
    ROS_ERROR("[NanFillerFilter] Check your layer type! Type %s does not exist", inputLayer_.c_str());
    return false;
  }

  // Add filtered layer
  mapOut.add(outputLayer_, mapIn[inputLayer_]);

  // Precompute value using setTo options
  float newValue = 0.0;
  if (setTo_ == "min") {
    newValue = mapOut.get(inputLayer_).minCoeffOfFinites();
    ROS_DEBUG_STREAM("[NanFillerFilter] newValue (min): " << newValue);

  } else if (setTo_ == "max") {
    newValue = mapOut.get(inputLayer_).maxCoeffOfFinites();
    ROS_DEBUG_STREAM("[NanFillerFilter] newValue (max): " << newValue);

  } else if (setTo_ == "mean") {
    newValue = mapOut.get(inputLayer_).meanOfFinites();
    ROS_DEBUG_STREAM("[NanFillerFilter] newValue (mean): " << newValue);

  } else if (setTo_ == "fixed_value") {
    newValue = value_;
    ROS_DEBUG_STREAM("[NanFillerFilter] newValue (fixed_value): " << value_);

  } else {
    ROS_WARN_STREAM("[NanFillerFilter] set_to option not valid. Using minimum by default");
    newValue = mapOut.get(inputLayer_).minCoeffOfFinites();
  }

  for (grid_map::GridMapIterator iterator(mapOut); !iterator.isPastEnd(); ++iterator) {
    // const size_t i = iterator.getLinearIndex();
    // ROS_WARN_STREAM("value at(" << i << ") before: " << mapOut.at(outputLayer_, *iterator));
    if (mapOut.isValid(*iterator, inputLayer_))
      mapOut.at(outputLayer_, *iterator) = mapOut.at(inputLayer_, *iterator);
    else
      mapOut.at(outputLayer_, *iterator) = newValue;
    // ROS_WARN_STREAM("value at(" << i << ") after : " << mapOut.at(outputLayer_, *iterator));
  }

  mapOut.setBasicLayers({});

  profiler_ptr_->endEvent("0.update");
  ROS_DEBUG_STREAM_THROTTLE(1, "-- Profiler report (throttled (1s)\n" << profiler_ptr_->getReport());

  return true;
}

}  // namespace grid_map

// Explicitly define the specialization for GridMap
template class grid_map::NanFillerFilter<grid_map::GridMap>;
// Export the filter.
PLUGINLIB_EXPORT_CLASS(grid_map::NanFillerFilter<grid_map::GridMap>, filters::FilterBase<grid_map::GridMap>)