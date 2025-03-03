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
 *  BaseHeightThresholdFilter.hpp
 *
 *  Author: Matias Mattamala
 */

#pragma once

#include <filters/filter_base.hpp>
#include <grid_map_filters_drs/utils/profiler.hpp>
#include <string>
#include <vector>

#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

namespace grid_map {

/*!
 * Applies a threshold to cells with specific height when transformed to a given frame
 */
template <typename T>
class BaseHeightThresholdFilter : public filters::FilterBase<T> {
 public:
  /*!
   * Constructor
   */
  BaseHeightThresholdFilter();

  /*!
   * Destructor.
   */
  virtual ~BaseHeightThresholdFilter();

  /*!
   * Configures the filter from parameters on the parameter server.
   */
  virtual bool configure();

  /*!
   * Uses either an upper or lower threshold. If the threshold is exceeded
   * the cell value is set to the predefined value set_to.
   * @param mapIn GridMap with the different layers to apply a threshold.
   * @param mapOut GridMap with the threshold applied to the layers.
   */
  virtual bool update(const T& mapIn, T& mapOut);

 private:
  //! Target frame
  std::string targetFrame_;
  //! Map frame
  std::string mapFrame_;
  //! Height threshold
  double heightThreshold_;
  //! Set to values
  double setToUpper_;
  double setToLower_;

  // TF listener
  std::shared_ptr<tf::TransformListener> tfListener_;

  // Layer names
  std::string inputLayer_;
  std::string outputLayer_;

  //! Profiler
  std::shared_ptr<Profiler> profiler_ptr_;
};

}  // namespace grid_map