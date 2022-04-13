/*
 * PointObstaclesFilter.hpp
 * 
 *  Author: Matias Mattamala
 */


#pragma once

#include <filters/filter_base.hpp>
#include <grid_map_filters_drs/utils/profiler.hpp>
#include <string>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>

#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_cv/grid_map_cv.hpp>

#include <costmap_converter/ObstacleMsg.h>
#include <costmap_converter/ObstacleArrayMsg.h>

#include <pluginlib/class_list_macros.h>

namespace grid_map {

/*!
 * Computes a signed distance field by applying a threshold on a layer
 */
template<typename T>
class PointObstaclesFilter : public filters::FilterBase<T>
{

 public:
  /*!
   * Constructor
   */
  PointObstaclesFilter();

  /*!
   * Destructor.
   */
  virtual ~PointObstaclesFilter();

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
  /*!
   * Helper to fill layers with cv::Mats
   */
  void addMatAsLayer(const cv::Mat& m, const std::string& layerName, grid_map::GridMap& gridMap, double resolution=1.0);

  grid_map::Position transformPosition(grid_map::Position& positionInOrigin, 
                                       const std::string& originFrame,
                                       const std::string& targetFrame);

  //! ROS helpers
  //! TF listener
  std::shared_ptr<tf::TransformListener> tfListener_;

  //! Layer the threshold will be evaluated.
  std::string inputLayer_;

  //! Output layer prefix
  std::string outputLayer_;

  //! Traversability threshold
  double threshold_;

  //! Attractor topic
  std::string obstaclesTopic_;

  //! Frame
  std::string obstaclesFrame_;

  //! Node handle
  ros::NodeHandle nodeHandle_;

  //! Obstacles subscriber
  ros::Publisher obstaclesPublisher_;

  //! Profiler
  std::shared_ptr<Profiler> profiler_ptr_;
};

} /* namespace */