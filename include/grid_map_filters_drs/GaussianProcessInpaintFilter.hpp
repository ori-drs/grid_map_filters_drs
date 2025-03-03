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
 * GaussianProcessInpaintFilter.hpp
 *
 *  Author: Matias Mattamala
 */

#pragma once

#include <filters/filter_base.hpp>
#include <grid_map_filters_drs/utils/profiler.hpp>
#include <limbo/limbo.hpp>
#include <string>
#include <vector>

using namespace limbo;

namespace grid_map {

/*!
 * Sets nan values to fixed value
 */
template <typename T>
class GaussianProcessInpaintFilter : public filters::FilterBase<T> {
  // Default kernel
  // Squared exponential
  struct Params {
    struct kernel : public defaults::kernel {
      BO_PARAM(double, noise, 1e-7);
      BO_PARAM(bool, optimize_noise, true);
    };
    struct kernel_exp {
      BO_PARAM(double, sigma_sq, 1.0);
      BO_PARAM(double, l, 10.0);
    };
    struct kernel_squared_exp_ard {
      BO_PARAM(double, sigma_sq, 1.0);
      BO_PARAM(int, k, 0);
    };
    struct kernel_maternfivehalves {
      /// @ingroup kernel_defaults
      BO_PARAM(double, sigma_sq, 1);
      /// @ingroup kernel_defaults
      BO_PARAM(double, l, 10);
    };
    struct kernel_maternthreehalves {
      /// @ingroup kernel_defaults
      BO_PARAM(double, sigma_sq, 1);
      /// @ingroup kernel_defaults
      BO_PARAM(double, l, 10);
    };
    struct opt_rprop : public defaults::opt_rprop {};
    // struct opt_nloptgrad : public defaults::opt_nloptgrad {
    // };
  };

 public:
  /*!
   * Constructor
   */
  GaussianProcessInpaintFilter();

  /*!
   * Destructor.
   */
  virtual ~GaussianProcessInpaintFilter();

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
  //! Layer the threshold will be evaluated.
  std::string inputLayer_;

  //! Output layer
  std::string outputLayer_;
  std::string outputLayerUpper_;
  std::string outputLayerLower_;
  std::string outputLayerMean_;

  //! Skips cells to reduce computation
  int subsampleSkip_;

  //! Profiler
  std::shared_ptr<Profiler> profiler_ptr_;
};

}  // namespace grid_map