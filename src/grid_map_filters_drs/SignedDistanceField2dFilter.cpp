/*
 * SignedDistanceField2dFilter.cpp
 *
 *  Implements a 2D SDF layer using a binary layer. 0 means obstacles, 1 free space
 * 
 *  Author: Matias Mattamala
 */


#include <grid_map_filters_drs/SignedDistanceField2dFilter.hpp>

using namespace filters;

namespace grid_map {

template<typename T>
SignedDistanceField2dFilter<T>::SignedDistanceField2dFilter()
{
}

template<typename T>
SignedDistanceField2dFilter<T>::~SignedDistanceField2dFilter()
{
}

template<typename T>
bool SignedDistanceField2dFilter<T>::configure()
{
  // Load Parameters
  // Input layer to be processed
  if (!FilterBase<T>::getParam(std::string("input_layer"), inputLayer_)) {
    ROS_ERROR("SDF 2D filter did not find parameter `input_layer`.");
    return false;
  }
  ROS_DEBUG("SDF 2D filter output_layer = %s.", inputLayer_.c_str());

  // Read output_layers_prefix, to define output grid map layers prefix.
  if (!filters::FilterBase<T>::getParam(std::string("output_layer"), outputLayer_)) {
    ROS_ERROR("SDF 2D filter did not find parameter `output_layer`.");
    return false;
  }
  ROS_DEBUG("SDF 2D filter output_layer = %s.", outputLayer_.c_str());

  // Read threshold, to define the untraversable areas
  if (!filters::FilterBase<T>::getParam(std::string("threshold"), threshold_)) {
    ROS_ERROR("SDF 2D filter did not find parameter `threshold`.");
    return false;
  }
  ROS_DEBUG("SDF 2D filter threshold = %f.", threshold_);

  // Read option to normalize gradients
  if (!filters::FilterBase<T>::getParam(std::string("normalize_gradients"), normalizeGradients_)) {
    ROS_ERROR("SDF 2D filter did not find parameter `normalize_gradients`.");
    return false;
  }
  ROS_DEBUG("SDF 2D filter normalize_gradients = %s.", (normalizeGradients_? "true" : "false"));

  return true;
}

template<typename T>
bool SignedDistanceField2dFilter<T>::update(const T& mapIn, T& mapOut)
{
  auto tic = std::chrono::high_resolution_clock::now();

  // Copy and fix indexing
  mapOut = mapIn;
  mapOut.convertToDefaultStartIndex();

  // Check if layer exists.
  if (!mapOut.exists(inputLayer_)) {
    ROS_ERROR("Check your threshold types! Type %s does not exist", inputLayer_.c_str());
    return false;
  }

  // Get resolution
  double resolution = mapOut.getResolution();

  // Convert selected layer to OpenCV image
  cv::Mat cvLayer;
  grid_map::GridMapCvConverter::toImage<float, 1>(mapOut, inputLayer_, CV_32F, cvLayer);

  // Apply threshold and compute obstacle masks
  cv::Mat cvObstacleSpace;
  cv::threshold(cvLayer, cvObstacleSpace, threshold_, 255, cv::THRESH_BINARY);	
  cvObstacleSpace.convertTo(cvObstacleSpace, CV_8UC1);
  cv::Mat cvFreeSpace = 255 - cvObstacleSpace;

  // Compute SDF
  cv::Mat cvSdfFreeSpace;
  cv::Mat cvSdfObstacleSpace;
  cv::distanceTransform(cvFreeSpace, cvSdfFreeSpace, cv::DIST_L2, 3); // TODO: parametrize kernel and distance metric
  cv::distanceTransform(cvObstacleSpace, cvSdfObstacleSpace, cv::DIST_L2, 3); // TODO: parametrize kernel and distance metric
  cv::Mat cvSdf = cvSdfObstacleSpace - cvSdfFreeSpace;

  cv::GaussianBlur(cvSdf, cvSdf, cv::Size(5, 5), 0);

  // Compute gradients
  cv::Mat cvGradientsX;
  cv::Mat cvGradientsY;
  cv::Mat cvGradientsZ(cvSdf.size(), cvSdf.type(), cv::Scalar(0.0));
  cv::Sobel(cvSdf, cvGradientsX, -1, 0, 1, 3);
  cv::Sobel(cvSdf, cvGradientsY, -1, 1, 0, 3);
  cvGradientsX*=resolution;
  cvGradientsY*=resolution;

  if(normalizeGradients_) {
    // Normalize gradients
    // Compute norm
    cv::Mat sqGradientsX;
    cv::Mat sqGradientsY;
    cv::pow(cvGradientsX, 2, sqGradientsX);
    cv::pow(cvGradientsY, 2, sqGradientsY);
    cv::Mat normNormal;
    cv::sqrt(sqGradientsX + sqGradientsY, normNormal);
    
    // Normalize
    cvGradientsX /= normNormal;
    cvGradientsY /= normNormal;
  }

  // Add layers
  addMatAsLayer(cvSdf, outputLayer_, mapOut, resolution);
  addMatAsLayer(cvGradientsX, outputLayer_+"_gradient_x", mapOut);
  addMatAsLayer(cvGradientsY, outputLayer_+"_gradient_y", mapOut);
  addMatAsLayer(cvGradientsZ, outputLayer_+"_gradient_z", mapOut);

  // cv::namedWindow("input layer", cv::WINDOW_NORMAL );
  // cv::imshow("input layer", cvInputLayer);
  // cv::waitKey(100);

  auto toc = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsedTime = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(toc - tic);
  ROS_DEBUG_STREAM("[SignedDistanceField2dFilter] Process time: " << elapsedTime.count() << " ms");

  return true;
}

template<typename T>
void SignedDistanceField2dFilter<T>::addMatAsLayer(const cv::Mat& mat, const std::string& layerName, grid_map::GridMap& gridMap, double resolution)
{
  // Get max and min
  double minDistance, maxDistance;
  cv::minMaxLoc(mat, &minDistance, &maxDistance);
  // ROS_WARN_STREAM("[" << layerName << "] mindist: " << minDistance << ", maxdist: " << maxDistance);

  minDistance*= resolution;
  maxDistance*= resolution;

  // cv::namedWindow(layerName + "_original", cv::WINDOW_NORMAL );
  // cv::imshow(layerName + "_original", mat);
  
  // Normalize sdf to get a greyscale image
  cv::Mat normalized;
  cv::normalize(mat, normalized, 0, 1.0, cv::NORM_MINMAX);
  normalized.convertTo(normalized, CV_32F);

  // cv::namedWindow(layerName + "_normalized", cv::WINDOW_NORMAL );
  // cv::imshow(layerName + "_normalized", normalized);

  // Add SDF layer to elevation map
  grid_map::GridMapCvConverter::addLayerFromImage<float, 1>(normalized, layerName,
                                                  gridMap, minDistance, maxDistance);
}

} /* namespace */

// Explicitly define the specialization for GridMap to have the filter implementation available for testing.
template class grid_map::SignedDistanceField2dFilter<grid_map::GridMap>;
// Export the filter.
PLUGINLIB_EXPORT_CLASS(grid_map::SignedDistanceField2dFilter<grid_map::GridMap>, filters::FilterBase<grid_map::GridMap>)