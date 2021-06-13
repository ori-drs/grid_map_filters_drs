/*
 * NanFillerFilter.cpp
 *
 *  Replaces NaNs by a fixed value
 * 
 *  Author: Matias Mattamala
 */



#include <grid_map_filters_drs/NanFillerFilter.hpp>
#include <grid_map_core/grid_map_core.hpp>
#include <pluginlib/class_list_macros.h>
#include <string>
#include <stdexcept>

using namespace filters;

namespace grid_map {

template<typename T>
NanFillerFilter<T>::NanFillerFilter()
    : setTo_("0.0")
{
}

template<typename T>
NanFillerFilter<T>::~NanFillerFilter()
{
}

template<typename T>
bool NanFillerFilter<T>::configure()
{
  // Load Parameters
  // Input layer to be processed
  if (!FilterBase<T>::getParam(std::string("input_layer"), inputLayer_)) {
    ROS_ERROR("NanFillerFilter filter did not find parameter `input_layer`.");
    return false;
  }
  ROS_DEBUG("NanFillerFilter filter input_layer = %s.", inputLayer_.c_str());

  // Output layer to be processed
  if (!FilterBase<T>::getParam(std::string("output_layer"), outputLayer_)) {
    ROS_ERROR("NanFillerFilter filter did not find parameter `output_layer`.");
    return false;
  }
  ROS_DEBUG("NanFillerFilter filter output_layer = %s.", outputLayer_.c_str());

  if (!FilterBase<T>::getParam(std::string("set_to"), setTo_)) {
    ROS_ERROR("NanFillerFilter did not find parameter 'set_to'.");
    return false;
  }
  ROS_DEBUG("NanFillerFilter filter set_to = %s.", setTo_.c_str());

  if (!FilterBase<T>::getParam(std::string("value"), value_)) {
    ROS_ERROR("NanFillerFilter did not find parameter 'set_to'.");
    return false;
  }
  ROS_DEBUG("NanFillerFilter filter value = %f.", value_);

  return true;
}

template<typename T>
bool NanFillerFilter<T>::update(const T& mapIn, T& mapOut)
{
  auto tic = std::chrono::high_resolution_clock::now();
  
  mapOut = mapIn;

  // Check if layer exists.
  if (!mapOut.exists(inputLayer_)) {
    ROS_ERROR("Check your layer type! Type %s does not exist", inputLayer_.c_str());
    return false;
  }

  // Add filtered layer
  mapOut.add(outputLayer_, mapIn[inputLayer_]);

  // Precompute value using setTo options
  float newValue = 0.0;
  if(setTo_ == "min") {
    newValue = mapOut.get(inputLayer_).minCoeffOfFinites();
    ROS_DEBUG_STREAM("newValue (min): " << newValue);

  } else if (setTo_ == "max") {
    newValue = mapOut.get(inputLayer_).maxCoeffOfFinites();
    ROS_DEBUG_STREAM("newValue (max): " << newValue);

  } else if (setTo_ == "mean") {
    newValue = mapOut.get(inputLayer_).meanOfFinites();
    ROS_DEBUG_STREAM("newValue (mean): " << newValue);

  } else if (setTo_ == "fixed_value") {
    newValue = value_;
    ROS_DEBUG_STREAM("newValue (fixed_value): " << value_);
  
  } else {
    ROS_WARN_STREAM("set_to option not valid. Using minimum by default");
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

  auto toc = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsedTime = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(toc - tic);
  ROS_DEBUG_STREAM("[NanFillerFilter] Process time: " << elapsedTime.count() << " ms");

  return true;
}

} /* namespace */

PLUGINLIB_EXPORT_CLASS(grid_map::NanFillerFilter<grid_map::GridMap>, filters::FilterBase<grid_map::GridMap>)