/*
 * DenoiseAndInpaintFilter.cpp
 *
 *  Based on original InpaintFilter 
 *  by Tanja Baumann, Peter Fankhauser (ETH Zurich, ANYbotics)
 * 
 *  Author: Matias Mattamala
 */


#include <grid_map_filters_drs/DenoiseAndInpaintFilter.hpp>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

// Grid Map
#include <grid_map_core/grid_map_core.hpp>

using namespace filters;

namespace grid_map {

template<typename T>
DenoiseAndInpaintFilter<T>::DenoiseAndInpaintFilter()
    : radius_(5.0) {

}

template<typename T>
DenoiseAndInpaintFilter<T>::~DenoiseAndInpaintFilter() {

}

template<typename T>
bool DenoiseAndInpaintFilter<T>::configure() {
  // Input layer
  if (!FilterBase < T > ::getParam(std::string("input_layer"), inputLayer_)) {
    ROS_ERROR("Inpaint filter did not find parameter `input_layer`.");
    return false;
  }
  ROS_DEBUG("Inpaint input layer is = %s.", inputLayer_.c_str());

  // Output layer
  if (!FilterBase < T > ::getParam(std::string("output_layer"), outputLayer_)) {
    ROS_ERROR("Inpaint filter did not find parameter `output_layer`.");
    return false;
  }
  ROS_DEBUG("Inpaint output layer = %s.", outputLayer_.c_str());

  // Inpainting radius
  if (!FilterBase < T > ::getParam(std::string("radius"), radius_)) {
    ROS_ERROR("InpaintRadius filter did not find param radius.");
    return false;
  }
  if (radius_ < 0.0) {
    ROS_ERROR("Radius must be greater than zero.");
    return false;
  }
  ROS_DEBUG("Radius = %f.", radius_);

  // Inpainting type
  inpaintingType_ = "ns";
  if (!FilterBase < T > ::getParam(std::string("inpainting_type"), inpaintingType_)) {
    ROS_WARN("Inpaint filter did not find parameter `inpainting_type`. Using default %s", inpaintingType_.c_str());
  }
  ROS_DEBUG("Inpaint inpainting_type = %s.", inpaintingType_.c_str());

  // Denoising options
  // Apply denoising flag
  applyDenoising_ = false;
  if (!FilterBase < T > ::getParam(std::string("pre_denoising"), applyDenoising_)) {
    ROS_WARN("Inpaint filter did not find parameter `pre_denoising`. Using default %s", (applyDenoising_? "true" : "false"));
  }
  ROS_DEBUG("Inpaint pre_denoising = %s.", (applyDenoising_? "true" : "false"));

  // Denoising radius
  denoisingRadius_ = 0.1;
  if (!FilterBase < T > ::getParam(std::string("denoising_radius"), denoisingRadius_)) {
    ROS_WARN("InpaintRadius filter did not find param denoising_radius. Using default %f", denoisingRadius_);
  }
  if (denoisingRadius_ < 0.0) {
    ROS_ERROR("denoising_radius must be greater than zero.");
    return false;
  }
  ROS_DEBUG("denoising_radius = %f.", denoisingRadius_);

  // Denoising type
  denoisingType_ = "median";
  if (!FilterBase < T > ::getParam(std::string("denoising_type"), denoisingType_)) {
    ROS_WARN("Inpaint filter did not find parameter `denoising_type`. Using default %s", denoisingType_.c_str());
  }
  ROS_DEBUG("Inpaint denoising_type = %s.", denoisingType_.c_str());

  return true;
}

template<typename T>
bool DenoiseAndInpaintFilter<T>::update(const T& mapIn, T& mapOut) {
  // Add new layer to the elevation map.
  mapOut = mapIn;
  mapOut.add(outputLayer_);

  //Convert elevation layer to OpenCV image to fill in holes.
  //Get the inpaint mask (nonzero pixels indicate where values need to be filled in).
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

  grid_map::GridMapCvConverter::toImage<unsigned char, 1>(mapOut, inputLayer_, CV_8UC1, minValue, maxValue,
                                                          originalImage);
  grid_map::GridMapCvConverter::toImage<unsigned char, 1>(mapOut, "inpaint_mask", CV_8UC1, mask);

  // Pre denoising
  if(applyDenoising_) {
    int radiusInPixels =  std::max((int)std::ceil(denoisingRadius_ / mapIn.getResolution()), 3); // Minimum kernel of size 3
    radiusInPixels = (radiusInPixels % 2 == 0)? radiusInPixels + 1 : radiusInPixels;

    ROS_WARN_STREAM("DENOISING RADIUS: " << radiusInPixels);

    if(denoisingType_ == "non_local") {
      const int searchWindowSize = 21; // (Odd number) Size in pixels of the window used to compute weighted average for given pixel
      const float h = 30; // Filter strength (larger numbers -> less noise but removes details)
      cv::fastNlMeansDenoising(originalImage, denoisedImage, h, radiusInPixels, searchWindowSize);	
    
    } else if(denoisingType_ == "total_variation" || denoisingType_ == "tv") {
      std::vector<cv::Mat> observations(1, originalImage);
      double lambda = 0.1;
      int nIters = 30;
      cv::denoise_TVL1(observations, denoisedImage, lambda, nIters);

    } else if(denoisingType_ == "gaussian") {
      cv::GaussianBlur(originalImage, denoisedImage, cv::Size(radiusInPixels, radiusInPixels), 0);
    
    } else if(denoisingType_ == "median") {
      cv::medianBlur(originalImage, denoisedImage, radiusInPixels);
    
    } else {
      ROS_WARN_STREAM("denoising_type option [" << denoisingType_ << "] not supported. Will not apply any deoising");
    }

    // Apply inverse mask to recover original input but denoised (to avoid artifacts on the edges)
    cv::Mat inverseMask;
    cv::bitwise_not(mask, inverseMask);
    cv::erode(inverseMask, inverseMask, cv::Mat());
    denoisedImage.copyTo(originalImage, inverseMask);
  }

  // Apply inpainting
  const double radiusInPixels = radius_ / mapIn.getResolution();
  if(inpaintingType_ == "telea") {
    cv::inpaint(originalImage, mask, filledImage, radiusInPixels, cv::INPAINT_TELEA);
  } else {
    // Navier-Stokes by default
    cv::inpaint(originalImage, mask, filledImage, radiusInPixels, cv::INPAINT_NS);
  }
  

  grid_map::GridMapCvConverter::addLayerFromImage<unsigned char, 3>(filledImage, outputLayer_, mapOut, minValue, maxValue);
  grid_map::GridMapCvConverter::addLayerFromImage<unsigned char, 3>(denoisedImage, "denoised", mapOut, minValue, maxValue);
  // mapOut.erase("inpaint_mask");

  return true;
}

}/* namespace */

PLUGINLIB_EXPORT_CLASS(grid_map::DenoiseAndInpaintFilter<grid_map::GridMap>, filters::FilterBase<grid_map::GridMap>)