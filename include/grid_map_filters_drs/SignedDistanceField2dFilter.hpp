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
 * SignedDistanceField2dFilter.hpp
 *
 *  Author: Matias Mattamala
 */

#pragma once

#include <filters/filter_base.hpp>
#include <grid_map_filters_drs/utils/profiler.hpp>
#include <string>
#include <vector>

#include <pluginlib/class_list_macros.h>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_cv/grid_map_cv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace grid_map {

/*!
 * Computes a signed distance field by applying a threshold on a layer
 */
template <typename T>
class SignedDistanceField2dFilter : public filters::FilterBase<T> {
 public:
  /*!
   * Constructor
   */
  SignedDistanceField2dFilter();

  /*!
   * Destructor.
   */
  virtual ~SignedDistanceField2dFilter();

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
  void addMatAsLayer(const cv::Mat& m, const std::string& layerName, grid_map::GridMap& gridMap, double resolution = 1.0);

  //! Layer the threshold will be evaluated.
  std::string inputLayer_;

  //! Output layer prefix
  std::string outputLayer_;

  //! Traversability threshold
  double threshold_;

  //! Flag to normalize output gradients
  bool normalizeGradients_;

  //! Profiler
  std::shared_ptr<Profiler> profiler_ptr_;
};

}  // namespace grid_map