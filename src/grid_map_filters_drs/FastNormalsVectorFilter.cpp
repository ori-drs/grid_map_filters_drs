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
 * FastNormalsVectorFilter.cpp
 *
 *  Fast normal computation using Sobel/Scharr filters
 *
 *  Author: Matias Mattamala
 */

#include <pluginlib/class_list_macros.h>
#include <Eigen/Dense>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_filters_drs/FastNormalsVectorFilter.hpp>
#include <stdexcept>
#include <string>

using namespace filters;

namespace grid_map {

template <typename T>
FastNormalsVectorFilter<T>::FastNormalsVectorFilter() {}

template <typename T>
FastNormalsVectorFilter<T>::~FastNormalsVectorFilter() {}

template <typename T>
bool FastNormalsVectorFilter<T>::configure() {
  // Setup profiler
  profiler_ptr_ = std::make_shared<Profiler>("FastNormalsVectorFilter");

  // Load Parameters
  // Input layer to be processed
  if (!FilterBase<T>::getParam(std::string("input_layer"), inputLayer_)) {
    ROS_ERROR("[FastNormalsVectorFilter] did not find parameter `input_layer`.");
    return false;
  }
  ROS_DEBUG("[FastNormalsVectorFilter] input_layers_prefix = %s.", inputLayer_.c_str());

  // Output layers to be processed
  if (!FilterBase<T>::getParam(std::string("output_layers_prefix"), outputLayersPrefix_)) {
    ROS_ERROR("[FastNormalsVectorFilter] did not find parameter `output_layers_prefix`.");
    return false;
  }
  ROS_DEBUG("[FastNormalsVectorFilter] output_layers_prefix = %s.", outputLayersPrefix_.c_str());

  // Pre smoothing
  // Use pre smoothing
  usePreSmoothing_ = true;
  if (!FilterBase<T>::getParam(std::string("use_pre_smoothing"), usePreSmoothing_)) {
    ROS_WARN("[FastNormalsVectorFilter] did not find parameter `use_pre_smoothing`. Using default %s",
             (usePreSmoothing_ ? "true" : "false"));
  }
  ROS_DEBUG("[FastNormalsVectorFilter] use_pre_smoothing = %s.", (usePreSmoothing_ ? "true" : "false"));

  // Radius to smooth the input layer
  preSmoothingRadius_ = 3;
  if (!FilterBase<T>::getParam(std::string("pre_smoothing_radius"), preSmoothingRadius_)) {
    ROS_WARN("[FastNormalsVectorFilter] did not find parameter `pre_smoothing_radius`.");
  }
  ROS_DEBUG("[FastNormalsVectorFilter] pre_smoothing_radius = %f.", preSmoothingRadius_);

  // Smoothing type
  preSmoothingType_ = "gaussian";
  if (!FilterBase<T>::getParam(std::string("pre_smoothing_type"), preSmoothingType_)) {
    ROS_WARN("[FastNormalsVectorFilter] did not find parameter `pre_smoothing_type`.");
  }
  ROS_DEBUG("[FastNormalsVectorFilter] pre_smoothing_type = %s.", preSmoothingType_.c_str());

  // Post smoothing
  // Use post smoothing
  usePostSmoothing_ = false;
  if (!FilterBase<T>::getParam(std::string("use_post_smoothing"), usePostSmoothing_)) {
    ROS_WARN("[FastNormalsVectorFilter] did not find parameter `use_post_smoothing`. Using default %s",
             (usePostSmoothing_ ? "true" : "false"));
  }
  ROS_DEBUG("[FastNormalsVectorFilter] use_post_smoothing = %s.", (usePostSmoothing_ ? "true" : "false"));

  // Radius to smooth the input layer
  postSmoothingRadius_ = 3;
  if (!FilterBase<T>::getParam(std::string("post_smoothing_radius"), postSmoothingRadius_)) {
    ROS_WARN("[FastNormalsVectorFilter] did not find parameter `post_smoothing_radius`.");
  }
  ROS_DEBUG("[FastNormalsVectorFilter] post_smoothing_radius = %f.", postSmoothingRadius_);

  // Smoothing type
  postSmoothingType_ = "median";
  if (!FilterBase<T>::getParam(std::string("post_smoothing_type"), postSmoothingType_)) {
    ROS_WARN("[FastNormalsVectorFilter] did not find parameter `post_smoothing_type`.");
  }
  ROS_DEBUG("[FastNormalsVectorFilter] post_smoothing_type = %s.", postSmoothingType_.c_str());

  // Preallocate normal layer names
  xOutputLayer_ = outputLayersPrefix_ + "x";
  yOutputLayer_ = outputLayersPrefix_ + "y";
  zOutputLayer_ = outputLayersPrefix_ + "z";

  return true;
}

template <typename T>
bool FastNormalsVectorFilter<T>::update(const T& mapIn, T& mapOut) {
  profiler_ptr_->startEvent("0.update");

  mapOut = mapIn;

  // Check if layer exists.
  if (!mapOut.exists(inputLayer_)) {
    ROS_ERROR("[FastNormalsVectorFilter] Check your layer type! Type %s does not exist", inputLayer_.c_str());
    return false;
  }

  // Get resolution
  double resolution = mapOut.getResolution();

  // Convert selected layer to OpenCV image
  cv::Mat cvLayer;
  const float minValue = mapOut.get(inputLayer_).minCoeffOfFinites();
  const float maxValue = mapOut.get(inputLayer_).maxCoeffOfFinites();

  grid_map::GridMapCvConverter::toImage<float, 1>(mapOut, inputLayer_, CV_32F, minValue, maxValue, cvLayer);
  cv::normalize(cvLayer, cvLayer, minValue, maxValue, cv::NORM_MINMAX);
  cvLayer.convertTo(cvLayer, CV_32F);

  // Pre smoothing
  if (usePreSmoothing_) {
    // Compute the smoothing radius
    int blurRadius = std::max((int)std::ceil(preSmoothingRadius_ / resolution), 3);
    blurRadius = (blurRadius % 2 == 0) ? blurRadius + 1 : blurRadius;

    if (preSmoothingType_ == "median") {
      cv::medianBlur(cvLayer, cvLayer, blurRadius);
    } else {
      // Blur before computing gradients to smooth the image
      cv::GaussianBlur(cvLayer, cvLayer, cv::Size(blurRadius, blurRadius), 0);
    }
  }

  // Compute gradients
  // Ref: https://stackoverflow.com/a/34644939/3570362
  float normalZ = 1.0;
  cv::Mat cvGradientsX(cvLayer.size(), cvLayer.type(), cv::Scalar(0.0));
  cv::Mat cvGradientsY(cvLayer.size(), cvLayer.type(), cv::Scalar(0.0));
  cv::Mat cvGradientsZ(cvLayer.size(), cvLayer.type(), cv::Scalar(normalZ));
  // cv::Sobel(cvLayer, cvGradientsX, -1, 0, 1, 3);
  // cv::Sobel(cvLayer, cvGradientsY, -1, 1, 0, 3);
  cv::Scharr(cvLayer, cvGradientsX, CV_32F, 0, 1, 3);
  cv::Scharr(cvLayer, cvGradientsY, CV_32F, 1, 0, 3);
  cvGradientsX /= 4;
  cvGradientsY /= 4;

  // Post smoothing
  if (usePostSmoothing_) {
    // Compute the smoothing radius
    int blurRadius = std::max((int)std::ceil(postSmoothingRadius_ / resolution), 3);
    blurRadius = (blurRadius % 2 == 0) ? blurRadius + 1 : blurRadius;

    if (postSmoothingType_ == "median") {
      cv::medianBlur(cvGradientsX, cvGradientsX, blurRadius);
      cv::medianBlur(cvGradientsY, cvGradientsY, blurRadius);
    } else {
      // Blur before computing gradients to smooth the image
      cv::GaussianBlur(cvGradientsX, cvGradientsX, cv::Size(blurRadius, blurRadius), 0);
      cv::GaussianBlur(cvGradientsY, cvGradientsY, cv::Size(blurRadius, blurRadius), 0);
    }
  }

  // Compute norm
  cv::Mat sqGradientsX;
  cv::Mat sqGradientsY;
  cv::pow(cvGradientsX, 2, sqGradientsX);
  cv::pow(cvGradientsY, 2, sqGradientsY);
  cv::Mat normNormal;
  cv::sqrt(sqGradientsX + sqGradientsY + normalZ * normalZ, normNormal);

  // Normalize
  cvGradientsX /= (normNormal + std::numeric_limits<float>::epsilon());
  cvGradientsY /= (normNormal + std::numeric_limits<float>::epsilon());
  cvGradientsZ /= (normNormal + std::numeric_limits<float>::epsilon());

  // Add layers
  addMatAsLayer(cvLayer, "normals_debug_input", mapOut);
  addMatAsLayer(cvGradientsX, xOutputLayer_, mapOut);
  addMatAsLayer(cvGradientsY, yOutputLayer_, mapOut);
  addMatAsLayer(cvGradientsZ, zOutputLayer_, mapOut);

  // Timing
  profiler_ptr_->endEvent("0.update");
  ROS_DEBUG_STREAM_THROTTLE(1, "-- Profiler report (throttled (1s)\n" << profiler_ptr_->getReport());

  return true;
}

template <typename T>
void FastNormalsVectorFilter<T>::addMatAsLayer(const cv::Mat& mat, const std::string& layerName, grid_map::GridMap& gridMap,
                                               double resolution) {
  // Get max and min
  double minDistance, maxDistance;
  cv::minMaxLoc(mat, &minDistance, &maxDistance);
  // ROS_WARN_STREAM("[" << layerName << "] mindist: " << minDistance << ", maxdist: " << maxDistance);

  minDistance *= resolution;
  maxDistance *= resolution;

  // Normalize values
  cv::Mat normalized;
  cv::normalize(mat, normalized, 0, 1.0, cv::NORM_MINMAX);
  normalized.convertTo(normalized, CV_32F);

  // Add SDF layer to elevation map
  grid_map::GridMapCvConverter::addLayerFromImage<float, 1>(normalized, layerName, gridMap, minDistance, maxDistance);
}

}  // namespace grid_map

// Explicitly define the specialization for GridMap
template class grid_map::FastNormalsVectorFilter<grid_map::GridMap>;
// Export the filter.
PLUGINLIB_EXPORT_CLASS(grid_map::FastNormalsVectorFilter<grid_map::GridMap>, filters::FilterBase<grid_map::GridMap>)