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
 * RadialInpaintFilter.hpp
 */

#pragma once

#include <filters/filter_base.hpp>

// OpenCV
#include <opencv2/opencv.hpp>
#include "grid_map_cv/grid_map_cv.hpp"

#include <string>
#include <vector>

namespace grid_map {

/*!
 * Uses OpenCV function to inpaint/fill holes in the input layer.
 */
template <typename T>
class RadialInpaintFilter : public filters::FilterBase<T> {
 public:
  /*!
   * Constructor
   */
  RadialInpaintFilter();

  /*!
   * Destructor.
   */
  virtual ~RadialInpaintFilter();

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
  //! Inpainting radius.
  double radius_;

  //! Input layer name.
  std::string inputLayer_;

  //! Output layer name.
  std::string outputLayer_;

  //! Aply denoising
  bool applyDenoising_;

  //! Denoising radius
  double denoisingRadius_;

  //! Non local parameters
  double nonLocalStrength_;
  int nonLocalSearchWindowSize_;
};

}  // namespace grid_map
