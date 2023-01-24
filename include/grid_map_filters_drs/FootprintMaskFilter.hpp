/*
 *  FootprintMaskFilter.hpp
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
 * It returns a mask with a footprint that can be used for other purposes in the filter chain
 */
template <typename T>
class FootprintMaskFilter : public filters::FilterBase<T> {
 public:
  /*!
   * Constructor
   */
  FootprintMaskFilter();

  /*!
   * Destructor.
   */
  virtual ~FootprintMaskFilter();

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
  // Layer names
  std::string outputLayer_;
  std::string footprintFrame_;
  std::string mapFrame_;
  std::string type_;
  double length_;
  double width_;
  double clearance_;

  // TF listener
  std::shared_ptr<tf::TransformListener> tfListener_;

  //! Profiler
  std::shared_ptr<Profiler> profiler_ptr_;
};

}  // namespace grid_map