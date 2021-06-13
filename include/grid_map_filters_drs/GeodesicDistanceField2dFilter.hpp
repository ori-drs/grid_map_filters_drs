/*
 * GeodesicDistanceField2dFilter.hpp
 * 
 *  Author: Matias Mattamala
 */


#pragma once

#include <filters/filter_base.hpp>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
#include <eigen_conversions/eigen_msg.h>
#include <std_srvs/Trigger.h>

#include <grid_map_filters_drs/thirdparty/GeodesicDistanceTransform.hpp>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_cv/grid_map_cv.hpp>
#include <pluginlib/class_list_macros.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace grid_map {

/*!
 * Computes a signed distance field by applying a threshold on a layer
 */
template<typename T>
class GeodesicDistanceField2dFilter : public filters::FilterBase<T>
{

 public:
  /*!
   * Constructor
   */
  GeodesicDistanceField2dFilter();

  /*!
   * Destructor.
   */
  virtual ~GeodesicDistanceField2dFilter();

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

  /*!
   * Subscribers the attractor from a pose message
   */
  void attractorCallback(const geometry_msgs::PoseStampedConstPtr& msg);

  //! Layer the threshold will be evaluated.
  std::string inputLayer_;

  //! Output layer prefix
  std::string outputLayer_;

  //! Traversability threshold
  double threshold_;
  bool useBinarization_;

  //! Attractor position
  grid_map::Position attractorPosition_;

  //! Attractor topic
  std::string attractorTopic_;

  //! Filter chain update service
  std::string filterChainUpdateService_;

  //! Target frame
  std::string attractorFrame_;
  //! Map frame
  std::string mapFrame_;
  //! Goal id
  ros::Time goalStamp_;

  //! Flag to normalize output gradients
  bool normalizeGradients_;

  //! ROS helpers
  //! TF listener
  std::shared_ptr<tf::TransformListener> tfListener_;

  //! Node handle
  ros::NodeHandle nodeHandle_;

  //! Attractor subscriber
  ros::Subscriber attractorSubscriber_;
  ros::Subscriber attractorSubscriberRviz_;
  ros::Subscriber attractorSubscriberRviz2_;

  //! Service client to force update of filter chain
  ros::ServiceClient forceUpdateClient_;
};

} /* namespace */