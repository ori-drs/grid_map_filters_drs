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
 * DeleteAllButFilter.hpp
 *
 *  Removes all layers but the ones on the list
 *
 *  Author: Matias Mattamala
 */

#pragma once

#include <algorithm>
#include <string>
#include <vector>

#include <filters/filter_base.hpp>

namespace grid_map {

/*!
 * Deletion filter class deletes layers of a grid map.
 */
template <typename T>
class DeleteAllButFilter : public filters::FilterBase<T> {
 public:
  /*!
   * Constructor
   */
  DeleteAllButFilter();

  /*!
   * Destructor.
   */
  virtual ~DeleteAllButFilter();

  /*!
   * Configures the filter from parameters on the parameter server.
   */
  virtual bool configure();

  /*!
   * Deletes the specified layers of a grid map.
   * @param mapIn gridMap with the different layers.
   * @param mapOut gridMap without the deleted layers.
   */
  virtual bool update(const T& mapIn, T& mapOut);

  /*!
   * Finds a layer in a list
   * @param layer query layer
   * @param keepLayers layers to be kept
   */
  bool exists(const std::string& layer, const std::vector<std::string>& keepLayers) const;

 private:
  //! List of layers that should be kept.
  std::vector<std::string> keepLayers_;
};

}  // namespace grid_map