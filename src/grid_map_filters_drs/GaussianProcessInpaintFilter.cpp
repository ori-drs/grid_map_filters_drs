/*
 * GaussianProcessInpaintFilter.cpp
 *
 *  Uses GPs to inpaint an elevation map. It requires the Limbo library
 * 
 *  Author: Matias Mattamala
 */



#include <grid_map_filters_drs/GaussianProcessInpaintFilter.hpp>
#include <grid_map_core/grid_map_core.hpp>
#include <pluginlib/class_list_macros.h>
#include <string>
#include <stdexcept>

using namespace filters;

namespace grid_map {

template<typename T>
GaussianProcessInpaintFilter<T>::GaussianProcessInpaintFilter()
    : subsampleSkip_(0)
{
}

template<typename T>
GaussianProcessInpaintFilter<T>::~GaussianProcessInpaintFilter()
{
}

template<typename T>
bool GaussianProcessInpaintFilter<T>::configure()
{
  // Setup profiler
  profiler_ptr_ = std::make_shared<Profiler>("GaussianProcessInpaintFilter");

  // Load Parameters
  // Input layer to be processed
  if (!FilterBase<T>::getParam(std::string("input_layer"), inputLayer_)) {
    ROS_ERROR("GaussianProcessInpaintFilter filter did not find parameter `input_layer`.");
    return false;
  }
  ROS_DEBUG("GaussianProcessInpaintFilter filter input_layer = %s.", inputLayer_.c_str());

  // Output layer to be processed
  if (!FilterBase<T>::getParam(std::string("output_layer"), outputLayer_)) {
    ROS_ERROR("GaussianProcessInpaintFilter filter did not find parameter `output_layer`.");
    return false;
  }
  ROS_DEBUG("GaussianProcessInpaintFilter filter output_layer = %s.", outputLayer_.c_str());

  // Read threshold, to define the untraversable areas
  double subSampleSkip = 0.0;
  if (!filters::FilterBase<T>::getParam(std::string("subsample_skip"), subSampleSkip)) {
    ROS_ERROR("Geodesic Distance 2D filter did not find parameter `subsample_skip`.");
    return false;
  }
  subsampleSkip_ = int(subSampleSkip);
  ROS_DEBUG("Geodesic Distance 2D filter subsample_skip = %i.", subsampleSkip_);

  outputLayerUpper_ = outputLayer_ + "_upper";
  outputLayerLower_ = outputLayer_ + "_lower";
  outputLayerMean_  = outputLayer_ + "_mean";
  

  return true;
}

template<typename T>
bool GaussianProcessInpaintFilter<T>::update(const T& mapIn, T& mapOut)
{
  profiler_ptr_->startEvent("0.update");
  
  mapOut = mapIn;

  // Check if layer exists.
  if (!mapOut.exists(inputLayer_)) {
    ROS_ERROR("Check your layer type! Type %s does not exist", inputLayer_.c_str());
    return false;
  }

  // Add filtered layer
  mapOut.add(outputLayer_, mapIn[inputLayer_]);
  mapOut.add(outputLayerUpper_, mapIn[inputLayer_]);
  mapOut.add(outputLayerLower_, mapIn[inputLayer_]);
  mapOut.add(outputLayerMean_,  mapIn[inputLayer_]);

  // Create GP
  // the type of the GP
  // using KernelType = kernel::Exp<Params>;
  // using KernelType = kernel::MaternThreeHalves<Params>;
  // using KernelType = kernel::MaternFiveHalves<Params>;
  using KernelType = kernel::SquaredExpARD<Params>;
  using MeanType   = mean::Data<Params>;

  using HyperParamOpt = model::gp::KernelLFOpt<Params>; // Optimize only kernel parameters
  // using HyperParamOpt = model::gp::KernelMeanLFOpt<Params>; // Optimize kernel parameters and mean
  // using HyperParamOpt = model::gp::MeanLFOpt<Params>; // Optimize only mean
  using GPOpt = model::GP<Params, KernelType, MeanType, HyperParamOpt>;
  using GP = model::GP<Params, KernelType, MeanType>;

  // Add samples to GP
  // 2D input, 1D output
  GPOpt gp(2, 1);

  grid_map::Size gridMapSize = mapIn.getSize();
  size_t numCells = gridMapSize(0) * gridMapSize(1); 

  std::vector<Eigen::VectorXd> samples;
  std::vector<Eigen::VectorXd> observations;
  samples.reserve(numCells);
  observations.reserve(numCells);

  ROS_WARN_STREAM("num cells      " << numCells);

  const grid_map::Matrix& data = mapIn[inputLayer_];
  for (grid_map::GridMapIterator iterator(mapOut); !iterator.isPastEnd(); ++iterator) {
    grid_map::Index index(*iterator);
    if (mapOut.isValid(index, inputLayer_)){
      if((index(0)% subsampleSkip_ != 0) || (index(1)% subsampleSkip_ != 0)){
        continue;
      }

      Eigen::VectorXd sample(2);
      sample << double(index(0)), double(index(1));
      samples.push_back(sample);
      
      Eigen::VectorXd obs(1);
      obs << data(index(0), index(1));
      observations.push_back(obs);
    }
  }

  ROS_WARN_STREAM("num samples      " << samples.size());
  ROS_WARN_STREAM("num observations " << observations.size());

  // Compute GP
  gp.compute(samples, observations);
  ROS_WARN_STREAM("after computing ");

  // Optimzie hyperparams
  gp.optimize_hyperparams();
  ROS_WARN_STREAM("after training ");

  // Write the predictions in the output layer
  grid_map::Matrix& dataOut = mapOut[outputLayer_];
  for (grid_map::GridMapIterator iterator(mapOut); !iterator.isPastEnd(); ++iterator) {
    grid_map::Index index(*iterator);

    // Create query index
    Eigen::VectorXd queryIndex(2);
    queryIndex << double(index(0)), double(index(1));

    // Query GP
    Eigen::VectorXd mu;
    double sigma;
    std::tie(mu, sigma) = gp.query(queryIndex);
    mapOut.at(outputLayerMean_, index) = mu(0);
    mapOut.at(outputLayerUpper_, index) = mu(0) + sigma;
    mapOut.at(outputLayerLower_, index) = mu(0) - sigma;

    if (!mapOut.isValid(index, inputLayer_)) {
      mapOut.at(outputLayer_, index) = mu(0);
    }

  }
   ROS_WARN_STREAM("after interpolating");

  mapOut.setBasicLayers({});

  profiler_ptr_->endEvent("0.update");
  ROS_DEBUG_STREAM_THROTTLE(1, "-- Profiler report (throttled (1s)\n" << profiler_ptr_->getReport());

  return true;
}

} /* namespace */

PLUGINLIB_EXPORT_CLASS(grid_map::GaussianProcessInpaintFilter<grid_map::GridMap>, filters::FilterBase<grid_map::GridMap>)