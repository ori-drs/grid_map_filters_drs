/*
 * DenoiseFilter.cpp
 *
 * Applies denoising algorithms (smoothing), such as Gaussian, Median, Total Variation, Bilateral
 *
 *  Author: Matias Mattamala
 */

#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <grid_map_filters_drs/DenoiseFilter.hpp>

// Grid Map
#include <grid_map_core/grid_map_core.hpp>

using namespace filters;

namespace grid_map {

template <typename T>
DenoiseFilter<T>::DenoiseFilter() : radius_(5.0) {}

template <typename T>
DenoiseFilter<T>::~DenoiseFilter() {}

template <typename T>
bool DenoiseFilter<T>::configure() {
  // Setup profiler
  profiler_ptr_ = std::make_shared<Profiler>("DenoiseFilter");

  // Input layer
  if (!FilterBase<T>::getParam(std::string("input_layer"), inputLayer_)) {
    ROS_ERROR("[DenoiseFilter] filter did not find parameter `input_layer`.");
    return false;
  }
  ROS_DEBUG("[DenoiseFilter] input layer is = %s.", inputLayer_.c_str());

  // Output layer
  if (!FilterBase<T>::getParam(std::string("output_layer"), outputLayer_)) {
    ROS_ERROR("[DenoiseFilter] did not find parameter `output_layer`.");
    return false;
  }
  ROS_DEBUG("[DenoiseFilter] output layer = %s.", outputLayer_.c_str());

  // Denoising radius
  radius_ = 0.1;
  if (!FilterBase<T>::getParam(std::string("radius"), radius_)) {
    ROS_WARN("[DenoiseFilter] filter did not find param radius. Using default %f", radius_);
  }
  if (radius_ < 0.0) {
    ROS_ERROR("[DenoiseFilter] radius must be greater than zero.");
    return false;
  }
  ROS_DEBUG("[DenoiseFilter] radius = %f.", radius_);

  // Denoising type
  type_ = "median";
  if (!FilterBase<T>::getParam(std::string("type"), type_)) {
    ROS_WARN("[DenoiseFilter] did not find parameter `type`. Using default %s", type_.c_str());
  }
  ROS_DEBUG("[DenoiseFilter] type = %s.", type_.c_str());

  // Total variation lambda
  totalVariationLambda_ = 0.5;
  if (!FilterBase<T>::getParam(std::string("total_variation_lambda"), totalVariationLambda_)) {
    ROS_WARN("[DenoiseFilter] did not find parameter `total_variation_lambda`. Using default %f", totalVariationLambda_);
  }
  ROS_DEBUG("[DenoiseFilter] total_variation_lambda = %f.", totalVariationLambda_);

  // Total variation iterations
  totalVariationIters_ = 30;
  if (!FilterBase<T>::getParam(std::string("total_variation_iters"), totalVariationIters_)) {
    ROS_WARN("[DenoiseFilter] did not find parameter `total_variation_iters`. Using default %i", totalVariationIters_);
  }
  ROS_DEBUG("[DenoiseFilter] total_variation_iters = %i.", totalVariationIters_);

  // Non local strength
  nonLocalStrength_ = 30;
  if (!FilterBase<T>::getParam(std::string("non_local_strength"), nonLocalStrength_)) {
    ROS_WARN("[DenoiseFilter] did not find parameter `non_local_strength`. Using default %f", nonLocalStrength_);
  }
  ROS_DEBUG("[DenoiseFilter] non_local_strength = %f.", nonLocalStrength_);

  // Non local search window size
  nonLocalSearchWindowSize_ = 21;
  if (!FilterBase<T>::getParam(std::string("non_local_search_window"), totalVariationIters_)) {
    ROS_WARN("[DenoiseFilter] did not find parameter `non_local_search_window`. Using default %i", nonLocalSearchWindowSize_);
  }
  ROS_DEBUG("[DenoiseFilter] non_local_search_window = %i.", nonLocalSearchWindowSize_);

  // Bilateral averaging radius
  bilateralWindowSize_ = 20;
  if (!FilterBase<T>::getParam(std::string("bilateral_window_size"), bilateralWindowSize_)) {
    ROS_WARN("[DenoiseFilter] did not find parameter `bilateral_window_size`. Using default %i", static_cast<int>(bilateralWindowSize_));
  }
  ROS_DEBUG("[DenoiseFilter] bilateral_window_size = %i.", static_cast<int>(bilateralWindowSize_));

  return true;
}

template <typename T>
bool DenoiseFilter<T>::update(const T& mapIn, T& mapOut) {
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

  // Convert layers to images
  cv::Mat originalImage;
  cv::Mat invalidMask;
  cv::Mat denoisedImage;
  const float minValue = mapOut.get(inputLayer_).minCoeffOfFinites();
  const float maxValue = mapOut.get(inputLayer_).maxCoeffOfFinites();

  grid_map::GridMapCvConverter::toImage<unsigned char, 1>(mapOut, inputLayer_, CV_8UC1, minValue, maxValue, originalImage);
  grid_map::GridMapCvConverter::toImage<unsigned char, 1>(mapOut, "invalid_mask", CV_8UC1, invalidMask);

  // Pre denoising
  int radiusInPixels = std::max((int)std::ceil(radius_ / mapIn.getResolution()), 3);  // Minimum kernel of size 3
  radiusInPixels = (radiusInPixels % 2 == 0) ? radiusInPixels + 1 : radiusInPixels;

  if (type_ == "non_local") {
    const int searchWindowSize = 21;  // (Odd number) Size in pixels of the window used to compute weighted average for given pixel
    const float h = 30;               // Filter strength (larger numbers -> less noise but removes details)
    cv::fastNlMeansDenoising(originalImage, denoisedImage, nonLocalStrength_, radiusInPixels, nonLocalSearchWindowSize_);

  } else if (type_ == "total_variation" || type_ == "tv") {
    std::vector<cv::Mat> observations(1, originalImage);
    cv::denoise_TVL1(observations, denoisedImage, totalVariationLambda_, totalVariationIters_);

  } else if (type_ == "gaussian") {
    cv::GaussianBlur(originalImage, denoisedImage, cv::Size(radiusInPixels, radiusInPixels), 0);

  } else if (type_ == "median") {
    cv::medianBlur(originalImage, denoisedImage, radiusInPixels);

  } else if (type_ == "bilateral") {
    cv::bilateralFilter(originalImage, denoisedImage, -1, bilateralWindowSize_, radiusInPixels);

  } else {
    ROS_WARN_STREAM("[DenoiseFilter] type option [" << type_ << "] not supported. Will not apply any denoising");
  }

  // Apply inverse mask to recover original input but denoised (to avoid artifacts on the edges)
  cv::Mat inverseMask;
  cv::bitwise_not(invalidMask, inverseMask);
  cv::erode(inverseMask, inverseMask, cv::Mat());
  denoisedImage.copyTo(originalImage, inverseMask);

  grid_map::GridMapCvConverter::addLayerFromImage<unsigned char, 1>(denoisedImage, outputLayer_, mapOut, minValue, maxValue);
  mapOut.erase("invalid_mask");

  // Timing
  profiler_ptr_->endEvent("0.update");
  ROS_DEBUG_STREAM_THROTTLE(1, "-- Profiler report (throttled (1s)\n" << profiler_ptr_->getReport());

  return true;
}

}  // namespace grid_map

// Explicitly define the specialization for GridMap
template class grid_map::DenoiseFilter<grid_map::GridMap>;
// Export the filter.
PLUGINLIB_EXPORT_CLASS(grid_map::DenoiseFilter<grid_map::GridMap>, filters::FilterBase<grid_map::GridMap>)