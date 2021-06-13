/*
 * FastNormalsVectorFilter.cpp
 *
 *  Fast normal computation using Sobel filters
 * 
 *  Author: Matias Mattamala
 */



#include <grid_map_filters_drs/FastNormalsVectorFilter.hpp>
#include <grid_map_core/grid_map_core.hpp>
#include <pluginlib/class_list_macros.h>
#include <string>
#include <stdexcept>
#include <Eigen/Dense>

using namespace filters;

namespace grid_map {

template<typename T>
FastNormalsVectorFilter<T>::FastNormalsVectorFilter()
{
}

template<typename T>
FastNormalsVectorFilter<T>::~FastNormalsVectorFilter()
{
}

template<typename T>
bool FastNormalsVectorFilter<T>::configure()
{
  // Load Parameters
  // Input layer to be processed
  if (!FilterBase<T>::getParam(std::string("input_layer"), inputLayer_)) {
    ROS_ERROR("FastNormalsVectorFilter filter did not find parameter `input_layer`.");
    return false;
  }
  ROS_DEBUG("FastNormalsVectorFilter filter input_layers_prefix = %s.", inputLayer_.c_str());

  // Output layers to be processed
  if (!FilterBase<T>::getParam(std::string("output_layers_prefix"), outputLayersPrefix_)) {
    ROS_ERROR("FastNormalsVectorFilter filter did not find parameter `output_layers_prefix`.");
    return false;
  }
  ROS_DEBUG("FastNormalsVectorFilter filter output_layers_prefix = %s.", outputLayersPrefix_.c_str());

  // Radius to smooth the input layer
  if (!FilterBase<T>::getParam(std::string("input_smoothing_radius"), inputSmoothingRadius_)) {
    ROS_ERROR("FastNormalsVectorFilter filter did not find parameter `input_smoothing_radius`.");
    return false;
  }
  ROS_DEBUG("FastNormalsVectorFilter filter output_layers_prefix = %f.", inputSmoothingRadius_);

  // Radius to smooth the normal layers
  if (!FilterBase<T>::getParam(std::string("normals_smoothing_radius"), normalsSmoothingRadius_)) {
    ROS_ERROR("FastNormalsVectorFilter filter did not find parameter `normals_smoothing_radius`.");
    return false;
  }
  ROS_DEBUG("FastNormalsVectorFilter filter output_layers_prefix = %f.", normalsSmoothingRadius_);

  // Preallocate normal layer names
  xOutputLayer_ = outputLayersPrefix_ + "x";
  yOutputLayer_ = outputLayersPrefix_ + "y";
  zOutputLayer_ = outputLayersPrefix_ + "z";

  return true;
}

template<typename T>
bool FastNormalsVectorFilter<T>::update(const T& mapIn, T& mapOut)
{
  auto tic = std::chrono::high_resolution_clock::now();

  mapOut = mapIn;

  // Check if layer exists.
  if (!mapOut.exists(inputLayer_)) {
    ROS_ERROR("Check your layer type! Type %s does not exist", inputLayer_.c_str());
    return false;
  }

  // Get resolution
  double resolution = mapOut.getResolution();

  // Convert selected layer to OpenCV image
  cv::Mat cvLayer;
  grid_map::GridMapCvConverter::toImage<float, 1>(mapOut, inputLayer_, CV_32F, cvLayer);

  // Compute the smoothing radius
  int inputBlurRadius = std::max((int)std::ceil(inputSmoothingRadius_ / resolution), 1);
  inputBlurRadius = (inputBlurRadius % 2 == 0)? inputBlurRadius + 1 : inputBlurRadius;

  // At least we use a smoothing of 3, and maximum 5 for float images
  int normalsBlurRadius = std::min(std::max((int)std::ceil(normalsSmoothingRadius_ / resolution), 1), 5); 
  normalsBlurRadius = (normalsBlurRadius % 2 == 0)? normalsBlurRadius + 1 : normalsBlurRadius;

  // Blur before computing gradients to smooth the image
  cv::GaussianBlur(cvLayer, cvLayer, cv::Size(inputBlurRadius, inputBlurRadius), 0);
  // cv::medianBlur(cvLayer, cvLayer, inputBlurRadius);

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
  // cvGradientsX*=resolution;
  // cvGradientsY*=resolution;
  cvGradientsX/=4;
  cvGradientsY/=4;

  // Statistical filter of gradients
  cv::medianBlur(cvGradientsX, cvGradientsX, normalsBlurRadius);
  cv::medianBlur(cvGradientsY, cvGradientsY, normalsBlurRadius);

  // Compute norm
  cv::Mat sqGradientsX;
  cv::Mat sqGradientsY;
  cv::pow(cvGradientsX, 2, sqGradientsX);
  cv::pow(cvGradientsY, 2, sqGradientsY);
  cv::Mat normNormal;
  cv::sqrt(sqGradientsX + sqGradientsY + normalZ*normalZ, normNormal);

  // Normalize
  cvGradientsX /= normNormal;
  cvGradientsY /= normNormal;
  cvGradientsZ /= normNormal;

  // Add layers
  addMatAsLayer(cvLayer, "normals_debug_input", mapOut);
  addMatAsLayer(cvGradientsX, xOutputLayer_, mapOut);
  addMatAsLayer(cvGradientsY, yOutputLayer_, mapOut);
  addMatAsLayer(cvGradientsZ, zOutputLayer_, mapOut);

  auto toc = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsedTime = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(toc - tic);
  ROS_DEBUG_STREAM("[FastNormalsVectorFilter] Process time: " << elapsedTime.count() << " ms");

  return true;
}

template<typename T>
void FastNormalsVectorFilter<T>::addMatAsLayer(const cv::Mat& mat, const std::string& layerName, grid_map::GridMap& gridMap, double resolution)
{
  // Get max and min
  double minDistance, maxDistance;
  cv::minMaxLoc(mat, &minDistance, &maxDistance);
  // ROS_WARN_STREAM("[" << layerName << "] mindist: " << minDistance << ", maxdist: " << maxDistance);

  minDistance*= resolution;
  maxDistance*= resolution;
  
  // Normalize values
  cv::Mat normalized;
  cv::normalize(mat, normalized, 0, 1.0, cv::NORM_MINMAX);
  normalized.convertTo(normalized, CV_32F);

  // Add SDF layer to elevation map
  grid_map::GridMapCvConverter::addLayerFromImage<float, 1>(normalized, layerName,
                                                  gridMap, minDistance, maxDistance);
}

} /* namespace */

PLUGINLIB_EXPORT_CLASS(grid_map::FastNormalsVectorFilter<grid_map::GridMap>, filters::FilterBase<grid_map::GridMap>)