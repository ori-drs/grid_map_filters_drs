/*
 * DenoiseAndInpaintFilter.cpp
 *
 *  Based on original InpaintFilter 
 *  by Tanja Baumann, Peter Fankhauser (ETH Zurich, ANYbotics)
 * 
 *  Author: Matias Mattamala
 */



#pragma once

#include <filters/filter_base.hpp>
#include <grid_map_filters_drs/utils/profiler.hpp>

//OpenCV
#include "grid_map_cv/grid_map_cv.hpp"
#include <opencv2/opencv.hpp>

#include <vector>
#include <string>

namespace grid_map {

/*!
 * Uses OpenCV function to inpaint/fill holes in the input layer.
 * It also enables to prefilter the image before inpainting
 */
template<typename T>
class DenoiseAndInpaintFilter : public filters::FilterBase<T> {

 public:
  /*!
   * Constructor
   */
  DenoiseAndInpaintFilter();

  /*!
   * Destructor.
   */
  virtual ~DenoiseAndInpaintFilter();

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

  //! Inpainting radius.
  double radius_;

  //! Inpainting type
  std::string inpaintingType_;

  //! Aply denoising
  bool applyDenoising_;

  //! Denoising radius
  double denoisingRadius_;

  //! Denoising type
  std::string denoisingType_;

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

} /* namespace */