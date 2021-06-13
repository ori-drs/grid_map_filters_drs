/*
 * FastNormalsVectorFilter.hpp
 *
 *  Author: Matias Mattamala
 */

#pragma once

#include <filters/filter_base.hpp>
#include <string>
#include <vector>

#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_cv/grid_map_cv.hpp>
#include <pluginlib/class_list_macros.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

namespace grid_map {

/*!
 * Rotates normal vectors to be consistent with base frame
 */
template<typename T>
class FastNormalsVectorFilter : public filters::FilterBase<T>
{

 public:
  /*!
   * Constructor
   */
  FastNormalsVectorFilter();

  /*!
   * Destructor.
   */
  virtual ~FastNormalsVectorFilter();

  /*!
   * Configures the filter from parameters on the parameter server.
   */
  virtual bool configure();

  /*!
   * Uses either an upper or lower threshold. If the threshold is exceeded
   * the cell value is set to the predefined value setTo_.
   * @param mapIn GridMap with the different layers to apply a threshold.
   * @param mapOut GridMap with the threshold applied to the layers.
   */
  virtual bool update(const T& mapIn, T& mapOut);

 private:
  /*!
   * Helper to fill layers with cv::Mats
   */
  void addMatAsLayer(const cv::Mat& m, const std::string& layerName, grid_map::GridMap& gridMap, double resolution=1.0);

  //! Input layer prefix for normals
  std::string inputLayer_;

  //! Output layers prefix
  std::string outputLayersPrefix_;

  //! Radius to smooth the normals
  double inputSmoothingRadius_;
  double normalsSmoothingRadius_;

  // Layer names
  std::string xOutputLayer_;
  std::string yOutputLayer_;
  std::string zOutputLayer_;
};

} /* namespace */