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
 * DenoiseFilter.cpp
 *
 * Applies denoising algorithms (smoothing), such as Gaussian, Median, Total Variation, Bilateral
 * 
 *  Author: Matias Mattamala
 */

#pragma once

#include <filters/filter_base.hpp>
#include <grid_map_filters_drs/utils/profiler.hpp>

// OpenCV
#include <opencv2/opencv.hpp>
#include "grid_map_cv/grid_map_cv.hpp"

#include <string>
#include <vector>

namespace grid_map {

/*!
 * Uses OpenCV function to inpaint/fill holes in the input layer.
 * It also enables to prefilter the image before inpainting
 */
template <typename T>
class DenoiseFilter : public filters::FilterBase<T> {
 public:
  /*!
   * Constructor
   */
  DenoiseFilter();

  /*!
   * Destructor.
   */
  virtual ~DenoiseFilter();

  /*!
   * Configures the filter from parameters on the Parameter Server
   */
  virtual bool configure();

  /*!
   * Adds a new output layer to the map.
   * Uses the OpenCV function inpaint holes in the input layer.
   * Saves to filled map in the outputlayer.
   * @param mapIn grid map containing input layer
   * @param mapOut grid map containing mapIn and inpainted input layer.
   */
  virtual bool update(const T& mapIn, T& mapOut);

 private:
  //! Input layer name.
  std::string inputLayer_;

  //! Output layer name.
  std::string outputLayer_;

  //! Denoising radius
  double radius_;

  //! Denoising type
  std::string type_;

  //! Total variation parameters
  double totalVariationLambda_;
  int totalVariationIters_;

  //! Non local parameters
  double nonLocalStrength_;
  int nonLocalSearchWindowSize_;

  //! Bilateral filtering
  double bilateralWindowSize_;

  //! Profiler
  std::shared_ptr<Profiler> profiler_ptr_;
};

}  // namespace grid_map