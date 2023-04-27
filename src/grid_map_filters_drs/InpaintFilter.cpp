/*
 * InpaintFilter.cpp
 *
 *  Based on original InpaintFilter
 *  by Tanja Baumann, Peter Fankhauser (ETH Zurich, ANYbotics)
 *
 *  It applies different inpainting algorithms
 *
 *  Author: Matias Mattamala
 */

#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <grid_map_filters_drs/InpaintFilter.hpp>

// Grid Map
#include <grid_map_core/grid_map_core.hpp>

using namespace filters;

namespace grid_map {

template <typename T>
InpaintFilter<T>::InpaintFilter() : radius_(5.0) {}

template <typename T>
InpaintFilter<T>::~InpaintFilter() {}

template <typename T>
bool InpaintFilter<T>::configure() {
  // Setup profiler
  profiler_ptr_ = std::make_shared<Profiler>("InpaintFilter");

  // Input layer
  if (!FilterBase<T>::getParam(std::string("input_layer"), inputLayer_)) {
    ROS_ERROR("[InpaintFilter] filter did not find parameter `input_layer`.");
    return false;
  }
  ROS_DEBUG("[InpaintFilter] input layer is = %s.", inputLayer_.c_str());

  // Output layer
  if (!FilterBase<T>::getParam(std::string("output_layer"), outputLayer_)) {
    ROS_ERROR("[InpaintFilter] did not find parameter `output_layer`.");
    return false;
  }
  ROS_DEBUG("[InpaintFilter] output layer = %s.", outputLayer_.c_str());

  // Inpainting radius
  if (!FilterBase<T>::getParam(std::string("radius"), radius_)) {
    ROS_ERROR("[InpaintFilter] filter did not find param radius.");
    return false;
  }
  if (radius_ < 0.0) {
    ROS_ERROR("[InpaintFilter] Radius must be greater than zero.");
    return false;
  }
  ROS_DEBUG("[InpaintFilter] Radius = %f.", radius_);

  // Inpainting type
  type_ = "ns";
  if (!FilterBase<T>::getParam(std::string("type"), type_)) {
    ROS_WARN("[InpaintFilter] did not find parameter `type`. Using default %s", type_.c_str());
  }
  ROS_DEBUG("[InpaintFilter] type = %s.", type_.c_str());

  return true;
}

template <typename T>
bool InpaintFilter<T>::update(const T& mapIn, T& mapOut) {
  profiler_ptr_->startEvent("0.update");

  // Add new layer to the elevation map.
  mapOut = mapIn;
  mapOut.add(outputLayer_);

    // Convert elevation layer to OpenCV image to fill in holes.
  // Get the inpaint mask (nonzero pixels indicate where values need to be filled in).
  mapOut.add("invalid_mask", 0.0);

  mapOut.setBasicLayers(std::vector<std::string>());
  for (grid_map::GridMapIterator iterator(mapOut); !iterator.isPastEnd(); ++iterator) {
    if (!mapOut.isValid(*iterator, inputLayer_)) {
      mapOut.at("invalid_mask", *iterator) = 1.0;
    }
  }

  cv::Mat originalImage;
  cv::Mat mask;
  cv::Mat filledImage;
  const float minValue = mapOut.get(inputLayer_).minCoeffOfFinites();
  const float maxValue = mapOut.get(inputLayer_).maxCoeffOfFinites();

  grid_map::GridMapCvConverter::toImage<unsigned char, 1>(mapOut, inputLayer_, CV_8UC1, minValue, maxValue, originalImage);
  grid_map::GridMapCvConverter::toImage<unsigned char, 1>(mapOut, "invalid_mask", CV_8UC1, mask);

  // Apply inpainting
  const double radiusInPixels = radius_ / mapIn.getResolution();
  if (type_ == "telea") {
    cv::inpaint(originalImage, mask, filledImage, radiusInPixels, cv::INPAINT_TELEA);
  } else {
    // Navier-Stokes by default
    cv::inpaint(originalImage, mask, filledImage, radiusInPixels, cv::INPAINT_NS);
  }

  // Add inpainted layer
  grid_map::GridMapCvConverter::addLayerFromImage<unsigned char, 1>(filledImage, outputLayer_, mapOut, minValue, maxValue);
  mapOut.erase("invalid_mask");

  // Timing
  profiler_ptr_->endEvent("0.update");
  ROS_DEBUG_STREAM_THROTTLE(1, "-- Profiler report (throttled (1s)\n" << profiler_ptr_->getReport());

  return true;
}

}  // namespace grid_map

// Explicitly define the specialization for GridMap
template class grid_map::InpaintFilter<grid_map::GridMap>;
// Export the filter.
PLUGINLIB_EXPORT_CLASS(grid_map::InpaintFilter<grid_map::GridMap>, filters::FilterBase<grid_map::GridMap>)