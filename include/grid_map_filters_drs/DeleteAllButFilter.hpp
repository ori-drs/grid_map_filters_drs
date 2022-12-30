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