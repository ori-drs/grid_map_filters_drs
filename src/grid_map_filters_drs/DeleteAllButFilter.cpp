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
 * DeleteAllButFilter.cpp
 *
 *  Removes all layers but the ones on the list
 *
 *  Author: Matias Mattamala
 */

#include <pluginlib/class_list_macros.h>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_filters_drs/DeleteAllButFilter.hpp>

using namespace filters;

namespace grid_map {

template <typename T>
DeleteAllButFilter<T>::DeleteAllButFilter() {}

template <typename T>
DeleteAllButFilter<T>::~DeleteAllButFilter() {}

template <typename T>
bool DeleteAllButFilter<T>::configure() {
  // Load Parameters
  if (!FilterBase<T>::getParam(std::string("layers"), keepLayers_)) {
    ROS_ERROR("DeleteAllButFilter did not find parameter 'layers'.");
    return false;
  }

  return true;
}

template <typename T>
bool DeleteAllButFilter<T>::update(const T& mapIn, T& mapOut) {
  mapOut = mapIn;

  for (const auto& layer : mapIn.getLayers()) {
    // Check if the layer is in the list of layers to keep
    if (!exists(layer, keepLayers_)) {
      if (mapOut.exists(layer)) {
        mapOut.erase(layer);
      }
    }
  }

  return true;
}

template <typename T>
bool DeleteAllButFilter<T>::exists(const std::string& layer, const std::vector<std::string>& keepLayers) const {
  const auto layerIterator = std::find(keepLayers.begin(), keepLayers.end(), layer);
  return layerIterator != keepLayers.end();
}

}  // namespace grid_map

// Explicitly define the specialization for GridMap
template class grid_map::DeleteAllButFilter<grid_map::GridMap>;
// Export the filter.
PLUGINLIB_EXPORT_CLASS(grid_map::DeleteAllButFilter<grid_map::GridMap>, filters::FilterBase<grid_map::GridMap>)