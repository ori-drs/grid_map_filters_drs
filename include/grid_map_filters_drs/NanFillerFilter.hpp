/*
 * NanFillerFilter.hpp
 *
 *  Author: Matias Mattamala
 */


#pragma once

#include <filters/filter_base.hpp>
#include <string>
#include <vector>

namespace grid_map {

/*!
 * Sets nan values to fixed value
 */
template<typename T>
class NanFillerFilter : public filters::FilterBase<T>
{

 public:
  /*!
   * Constructor
   */
  NanFillerFilter();

  /*!
   * Destructor.
   */
  virtual ~NanFillerFilter();

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

  //! Set to
  std::string setTo_;

  //! Value to set the nan values (if selected)
  double value_; 
};

} /* namespace */