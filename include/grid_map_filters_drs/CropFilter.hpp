/*
 * CropFilter.hpp
 *
 *  Crops the full grid map into a smaller size
 *
 *  Author: Matias Mattamala
 */


#pragma once

#include <filters/filter_base.hpp>
#include <grid_map_filters_drs/utils/profiler.hpp>
#include <string>
#include <vector>

namespace grid_map {

/*!
 * Sets nan values to fixed value
 */
template <typename T>
class CropFilter : public filters::FilterBase<T> {
 public:
  /*!
   * Constructor
   */
  CropFilter();

  /*!
   * Destructor.
   */
  virtual ~CropFilter();

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
  grid_map::Position crop_center_;
  grid_map::Length crop_length_;

  //! Profiler
  std::shared_ptr<Profiler> profiler_ptr_;
};

}  // namespace grid_map