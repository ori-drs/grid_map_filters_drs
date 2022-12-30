/*
 * DenoiseAndInpaintFilter.cpp
 *
 *  Based on original InpaintFilter
 *  by Tanja Baumann, Peter Fankhauser (ETH Zurich, ANYbotics)
 *
 *  It applies different denoising and inpainting algorithms
 *
 *  Author: Matias Mattamala
 */

#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <grid_map_filters_drs/DenoiseAndInpaintFilter.hpp>

// Grid Map
#include <grid_map_core/grid_map_core.hpp>

using namespace filters;

namespace grid_map {

template <typename T>
DenoiseAndInpaintFilter<T>::DenoiseAndInpaintFilter() : radius_(5.0) {}

template <typename T>
DenoiseAndInpaintFilter<T>::~DenoiseAndInpaintFilter() {}

template <typename T>
bool DenoiseAndInpaintFilter<T>::configure() {
  // Setup profiler
  profiler_ptr_ = std::make_shared<Profiler>("DenoiseAndInpaintFilter");

  // Input layer
  if (!FilterBase<T>::getParam(std::string("input_layer"), inputLayer_)) {
    ROS_ERROR("[DenoiseAndInpaintFilter] filter did not find parameter `input_layer`.");
    return false;
  }
  ROS_DEBUG("[DenoiseAndInpaintFilter] input layer is = %s.", inputLayer_.c_str());

  // Output layer
  if (!FilterBase<T>::getParam(std::string("output_layer"), outputLayer_)) {
    ROS_ERROR("[DenoiseAndInpaintFilter] did not find parameter `output_layer`.");
    return false;
  }
  ROS_DEBUG("[DenoiseAndInpaintFilter] output layer = %s.", outputLayer_.c_str());

  // Inpainting radius
  if (!FilterBase<T>::getParam(std::string("radius"), radius_)) {
    ROS_ERROR("[DenoiseAndInpaintFilter] filter did not find param radius.");
    return false;
  }
  if (radius_ < 0.0) {
    ROS_ERROR("[DenoiseAndInpaintFilter] Radius must be greater than zero.");
    return false;
  }
  ROS_DEBUG("[DenoiseAndInpaintFilter] Radius = %f.", radius_);

  // Inpainting type
  inpaintingType_ = "ns";
  if (!FilterBase<T>::getParam(std::string("inpainting_type"), inpaintingType_)) {
    ROS_WARN("[DenoiseAndInpaintFilter] did not find parameter `inpainting_type`. Using default %s", inpaintingType_.c_str());
  }
  ROS_DEBUG("[DenoiseAndInpaintFilter] inpainting_type = %s.", inpaintingType_.c_str());

  // Denoising options
  // Apply denoising flag
  applyDenoising_ = false;
  if (!FilterBase<T>::getParam(std::string("pre_denoising"), applyDenoising_)) {
    ROS_WARN("[DenoiseAndInpaintFilter] did not find parameter `pre_denoising`. Using default %s", (applyDenoising_ ? "true" : "false"));
  }
  ROS_DEBUG("[DenoiseAndInpaintFilter] pre_denoising = %s.", (applyDenoising_ ? "true" : "false"));

  // Denoising radius
  denoisingRadius_ = 0.1;
  if (!FilterBase<T>::getParam(std::string("denoising_radius"), denoisingRadius_)) {
    ROS_WARN("[DenoiseAndInpaintFilter] filter did not find param denoising_radius. Using default %f", denoisingRadius_);
  }
  if (denoisingRadius_ < 0.0) {
    ROS_ERROR("[DenoiseAndInpaintFilter] denoising_radius must be greater than zero.");
    return false;
  }
  ROS_DEBUG("[DenoiseAndInpaintFilter] denoising_radius = %f.", denoisingRadius_);

  // Denoising type
  denoisingType_ = "median";
  if (!FilterBase<T>::getParam(std::string("denoising_type"), denoisingType_)) {
    ROS_WARN("[DenoiseAndInpaintFilter] did not find parameter `denoising_type`. Using default %s", denoisingType_.c_str());
  }
  ROS_DEBUG("[DenoiseAndInpaintFilter] denoising_type = %s.", denoisingType_.c_str());

  // Total variation lambda
  totalVariationLambda_ = 0.5;
  if (!FilterBase<T>::getParam(std::string("total_variation_lambda"), totalVariationLambda_)) {
    ROS_WARN("[DenoiseAndInpaintFilter] did not find parameter `total_variation_lambda`. Using default %f", totalVariationLambda_);
  }
  ROS_DEBUG("[DenoiseAndInpaintFilter] total_variation_lambda = %f.", totalVariationLambda_);

  // Total variation iterations
  totalVariationIters_ = 30;
  if (!FilterBase<T>::getParam(std::string("total_variation_iters"), totalVariationIters_)) {
    ROS_WARN("[DenoiseAndInpaintFilter] did not find parameter `total_variation_iters`. Using default %i", totalVariationIters_);
  }
  ROS_DEBUG("[DenoiseAndInpaintFilter] total_variation_iters = %i.", totalVariationIters_);

  // Non local strength
  nonLocalStrength_ = 30;
  if (!FilterBase<T>::getParam(std::string("non_local_strength"), nonLocalStrength_)) {
    ROS_WARN("[DenoiseAndInpaintFilter] did not find parameter `non_local_strength`. Using default %f", nonLocalStrength_);
  }
  ROS_DEBUG("[DenoiseAndInpaintFilter] non_local_strength = %f.", nonLocalStrength_);

  // Non local search window size
  nonLocalSearchWindowSize_ = 21;
  if (!FilterBase<T>::getParam(std::string("non_local_search_window"), totalVariationIters_)) {
    ROS_WARN("[DenoiseAndInpaintFilter] did not find parameter `non_local_search_window`. Using default %i", nonLocalSearchWindowSize_);
  }
  ROS_DEBUG("[DenoiseAndInpaintFilter] non_local_search_window = %i.", nonLocalSearchWindowSize_);

  // Bilateral averaging radius
  bilateralWindowSize_ = 20;
  if (!FilterBase<T>::getParam(std::string("bilateral_window_size"), bilateralWindowSize_)) {
    ROS_WARN("[DenoiseAndInpaintFilter] did not find parameter `bilateral_window_size`. Using default %i",
             static_cast<int>(bilateralWindowSize_));
  }
  ROS_DEBUG("[DenoiseAndInpaintFilter] bilateral_window_size = %i.", static_cast<int>(bilateralWindowSize_));

  return true;
}

template <typename T>
bool DenoiseAndInpaintFilter<T>::update(const T& mapIn, T& mapOut) {
  profiler_ptr_->startEvent("0.update");

  // Add new layer to the elevation map.
  mapOut = mapIn;
  mapOut.add(outputLayer_);

  // Convert elevation layer to OpenCV image to fill in holes.
  // Get the inpaint mask (nonzero pixels indicate where values need to be filled in).
  mapOut.add("inpaint_mask", 0.0);

  mapOut.setBasicLayers(std::vector<std::string>());
  for (grid_map::GridMapIterator iterator(mapOut); !iterator.isPastEnd(); ++iterator) {
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

  grid_map::GridMapCvConverter::toImage<unsigned char, 1>(mapOut, inputLayer_, CV_8UC1, minValue, maxValue, originalImage);
  grid_map::GridMapCvConverter::toImage<unsigned char, 1>(mapOut, "inpaint_mask", CV_8UC1, mask);

  // Pre denoising
  if (applyDenoising_) {
    int radiusInPixels = std::max((int)std::ceil(denoisingRadius_ / mapIn.getResolution()), 3);  // Minimum kernel of size 3
    radiusInPixels = (radiusInPixels % 2 == 0) ? radiusInPixels + 1 : radiusInPixels;

    // ROS_WARN_STREAM("[DenoiseAndInpaintFilter] denoising radius: " << radiusInPixels);

    if (denoisingType_ == "non_local") {
      const int searchWindowSize = 21;  // (Odd number) Size in pixels of the window used to compute weighted average for given pixel
      const float h = 30;               // Filter strength (larger numbers -> less noise but removes details)
      cv::fastNlMeansDenoising(originalImage, denoisedImage, nonLocalStrength_, radiusInPixels, nonLocalSearchWindowSize_);

    } else if (denoisingType_ == "total_variation" || denoisingType_ == "tv") {
      std::vector<cv::Mat> observations(1, originalImage);
      cv::denoise_TVL1(observations, denoisedImage, totalVariationLambda_, totalVariationIters_);

    } else if (denoisingType_ == "gaussian") {
      cv::GaussianBlur(originalImage, denoisedImage, cv::Size(radiusInPixels, radiusInPixels), 0);

    } else if (denoisingType_ == "median") {
      cv::medianBlur(originalImage, denoisedImage, radiusInPixels);

    } else if (denoisingType_ == "bilateral") {
      cv::bilateralFilter(originalImage, denoisedImage, -1, bilateralWindowSize_, radiusInPixels);

    } else {
      ROS_WARN_STREAM("[DenoiseAndInpaintFilter] denoising_type option [" << denoisingType_
                                                                          << "] not supported. Will not apply any deoising");
    }

    // Apply inverse mask to recover original input but denoised (to avoid artifacts on the edges)
    cv::Mat inverseMask;
    cv::bitwise_not(mask, inverseMask);
    cv::erode(inverseMask, inverseMask, cv::Mat());
    denoisedImage.copyTo(originalImage, inverseMask);

    grid_map::GridMapCvConverter::addLayerFromImage<unsigned char, 1>(denoisedImage, "denoised", mapOut, minValue, maxValue);
  }

  // Apply inpainting
  const double radiusInPixels = radius_ / mapIn.getResolution();
  if (inpaintingType_ == "telea") {
    cv::inpaint(originalImage, mask, filledImage, radiusInPixels, cv::INPAINT_TELEA);
  } else {
    // Navier-Stokes by default
    cv::inpaint(originalImage, mask, filledImage, radiusInPixels, cv::INPAINT_NS);
  }

  // Add inpainted layer
  grid_map::GridMapCvConverter::addLayerFromImage<unsigned char, 1>(filledImage, outputLayer_, mapOut, minValue, maxValue);
  mapOut.erase("inpaint_mask");

  // Timing
  profiler_ptr_->endEvent("0.update");
  ROS_DEBUG_STREAM_THROTTLE(1, "-- Profiler report (throttled (1s)\n" << profiler_ptr_->getReport());

  return true;
}

}  // namespace grid_map

// Explicitly define the specialization for GridMap
template class grid_map::DenoiseAndInpaintFilter<grid_map::GridMap>;
// Export the filter.
PLUGINLIB_EXPORT_CLASS(grid_map::DenoiseAndInpaintFilter<grid_map::GridMap>, filters::FilterBase<grid_map::GridMap>)