/*
 * RadialInpaintFilter.cpp
 *
 *  Based on original InpaintFilter
 *  by Tanja Baumann, Peter Fankhauser (ETH Zurich, ANYbotics)
 *
 *  Author: Oliwier Melon
 *  Author: Wolfgang Merkt
 *
 *  Note: Requires testing
 */

#include "grid_map_filters_drs/RadialInpaintFilter.hpp"
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

// Grid Map
#include <grid_map_core/grid_map_core.hpp>

using namespace filters;

namespace grid_map {

template <typename T>
RadialInpaintFilter<T>::RadialInpaintFilter() : radius_(5.0) {}

template <typename T>
RadialInpaintFilter<T>::~RadialInpaintFilter() {}

template <typename T>
bool RadialInpaintFilter<T>::configure() {
  if (!FilterBase<T>::getParam(std::string("radius"), radius_)) {
    ROS_ERROR("InpaintRadius filter did not find param radius.");
    return false;
  }

  if (radius_ < 0.0) {
    ROS_ERROR("Radius must be greater than zero.");
    return false;
  }

  ROS_DEBUG("Radius = %f.", radius_);

  if (!FilterBase<T>::getParam(std::string("input_layer"), inputLayer_)) {
    ROS_ERROR("Inpaint filter did not find parameter `input_layer`.");
    return false;
  }

  ROS_DEBUG("Inpaint input layer is = %s.", inputLayer_.c_str());

  if (!FilterBase<T>::getParam(std::string("output_layer"), outputLayer_)) {
    ROS_ERROR("Inpaint filter did not find parameter `output_layer`.");
    return false;
  }

  ROS_DEBUG("Inpaint output layer = %s.", outputLayer_.c_str());

  // Denoising options
  // Apply denoising flag
  applyDenoising_ = false;
  if (!FilterBase<T>::getParam(std::string("pre_denoising"), applyDenoising_)) {
    ROS_WARN("[RadialInpaintFilter] did not find parameter `pre_denoising`. Using default %s", (applyDenoising_ ? "true" : "false"));
  }
  ROS_DEBUG("[RadialInpaintFilter] pre_denoising = %s.", (applyDenoising_ ? "true" : "false"));

  // Denoising radius
  denoisingRadius_ = 0.1;
  if (!FilterBase<T>::getParam(std::string("denoising_radius"), denoisingRadius_)) {
    ROS_WARN("[RadialInpaintFilter] filter did not find param denoising_radius. Using default %f", denoisingRadius_);
  }
  if (denoisingRadius_ < 0.0) {
    ROS_ERROR("[RadialInpaintFilter] denoising_radius must be greater than zero.");
    return false;
  }
  ROS_DEBUG("[RadialInpaintFilter] denoising_radius = %f.", denoisingRadius_);

  // Non local strength
  nonLocalStrength_ = 30;
  if (!FilterBase<T>::getParam(std::string("non_local_strength"), nonLocalStrength_)) {
    ROS_WARN("[RadialInpaintFilter] did not find parameter `non_local_strength`. Using default %f", nonLocalStrength_);
  }
  ROS_DEBUG("[RadialInpaintFilter] non_local_strength = %f.", nonLocalStrength_);

  // Non local search window size
  nonLocalSearchWindowSize_ = 21;
  if (!FilterBase<T>::getParam(std::string("non_local_search_window"), nonLocalSearchWindowSize_)) {
    ROS_WARN("[RadialInpaintFilter] did not find parameter `non_local_search_window`. Using default %i", nonLocalSearchWindowSize_);
  }
  ROS_DEBUG("[RadialInpaintFilter] non_local_search_window = %i.", nonLocalSearchWindowSize_);

  return true;
}

template <typename T>
bool RadialInpaintFilter<T>::update(const T& mapIn, T& mapOut) {
  // Add new layer to the elevation map.
  mapOut = mapIn;
  mapOut.add(outputLayer_);
  mapOut.add("elevation_original", mapOut.get("elevation"));

  // Convert elevation layer to OpenCV image to fill in holes.
  // Get the inpaint mask (nonzero pixels indicate where values need to be filled in).
  mapOut.add("inpaint_mask", 0.0);

  mapOut.setBasicLayers(std::vector<std::string>());
  for (grid_map::GridMapIterator iterator(mapOut); !iterator.isPastEnd(); ++iterator) {
    if (mapOut.exists("upper_bound") && mapOut.exists("lower_bound")) {
      if (mapOut.at("upper_bound", *iterator) - mapOut.at("lower_bound", *iterator) > 0.1) {
        mapOut.at(inputLayer_, *iterator) = std::numeric_limits<double>::quiet_NaN();
      }
    }
    if (!mapOut.isValid(*iterator, inputLayer_)) {
      mapOut.at("inpaint_mask", *iterator) = 1.0;
    }
  }
  cv::Mat originalImage;
  cv::Mat mask;
  cv::Mat filledImage;
  cv::Mat denoisedImage;
  const float minValue = mapOut.get(inputLayer_).minCoeffOfFinites();
  const float maxValue = mapOut.get(inputLayer_).maxCoeffOfFinites();

  grid_map::GridMapCvConverter::toImage<unsigned char, 3>(mapOut, inputLayer_, CV_8UC3, minValue, maxValue, originalImage);
  grid_map::GridMapCvConverter::toImage<unsigned char, 1>(mapOut, "inpaint_mask", CV_8UC1, mask);

  if (applyDenoising_) {
    int radiusInPixelsDenoising = std::max((int)std::ceil(denoisingRadius_ / mapIn.getResolution()), 3);  // Minimum kernel of size 3
    radiusInPixelsDenoising = (radiusInPixelsDenoising % 2 == 0) ? radiusInPixelsDenoising + 1 : radiusInPixelsDenoising;
    cv::fastNlMeansDenoising(originalImage, denoisedImage, nonLocalStrength_, radiusInPixelsDenoising, nonLocalSearchWindowSize_);

    // Apply inverse mask to recover original input but denoised (to avoid artifacts on the edges)
    cv::Mat inverseMask;
    cv::bitwise_not(mask, inverseMask);
    cv::erode(inverseMask, inverseMask, cv::Mat());
    denoisedImage.copyTo(originalImage, inverseMask);

    grid_map::GridMapCvConverter::addLayerFromImage<unsigned char, 1>(denoisedImage, "denoised", mapOut, minValue, maxValue);
  }

  const double radiusInPixels = radius_ / mapIn.getResolution();
  cv::inpaint(originalImage, mask, filledImage, radiusInPixels, cv::INPAINT_NS);

  grid_map::GridMapCvConverter::addLayerFromImage<unsigned char, 3>(filledImage, "cv_inpainting", mapOut, minValue, maxValue);
  mapOut.erase("inpaint_mask");

  // radial
  grid_map::Position gridCenterPosition = mapOut.getPosition();
  mapOut.add(outputLayer_, mapOut.get(inputLayer_));

  for (grid_map::GridMapIterator iterator(mapOut); !iterator.isPastEnd(); ++iterator) {
    if (!mapOut.isValid(*iterator, outputLayer_)) {
      grid_map::Position position;
      mapOut.getPosition(*iterator, position);  // add safety;

      grid_map::Position step = position - gridCenterPosition;
      step = step / step.norm() * mapOut.getResolution();
      grid_map::Position line_search_position = position + step;
      grid_map::Index index;
      bool success = false;
      float forward = std::numeric_limits<double>::infinity();
      float backward = std::numeric_limits<double>::infinity();

      while (mapOut.isInside(line_search_position)) {
        mapOut.getIndex(line_search_position, index);
        if (mapOut.isValid(index, inputLayer_)) {
          forward = mapOut.at(outputLayer_, index);
          success = true;
          break;
        }
        line_search_position += step;
      }
      line_search_position = position - step;
      while (mapOut.isInside(line_search_position)) {
        mapOut.getIndex(line_search_position, index);
        if (mapOut.isValid(index, inputLayer_)) {
          backward = mapOut.at(outputLayer_, index);
          success = true;
          break;
        }
        line_search_position -= step;
      }
      if (success) {
        mapOut.at(outputLayer_, *iterator) = std::min(forward, backward);
      } else {
        mapOut.at(outputLayer_, *iterator) = mapOut.at("cv_inpainting", *iterator);
      }
    }
  }

  return true;
}

}  // namespace grid_map

// Explicitly define the specialization for GridMap to have the filter implementation available for testing.
template class grid_map::RadialInpaintFilter<grid_map::GridMap>;
// Export the filter.
PLUGINLIB_EXPORT_CLASS(grid_map::RadialInpaintFilter<grid_map::GridMap>, filters::FilterBase<grid_map::GridMap>)
