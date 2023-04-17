/*
 * GeodesicDistanceField2dFilter.cpp
 *
 *  Implements a Geodesic Distance layer using a binary layer. 0 means obstacles, 1 free space
 *
 *  Author: Matias Mattamala
 */

#include <grid_map_filters_drs/GeodesicDistanceField2dFilter.hpp>

using namespace filters;

namespace grid_map {

template <typename T>
GeodesicDistanceField2dFilter<T>::GeodesicDistanceField2dFilter() : attractorPosition_(0, 0), attractorStamp_(0), mapFrame_("not_set") {}

template <typename T>
GeodesicDistanceField2dFilter<T>::~GeodesicDistanceField2dFilter() {}

template <typename T>
bool GeodesicDistanceField2dFilter<T>::configure() {
  // Setup profiler
  profiler_ptr_ = std::make_shared<Profiler>("GeodesicDistanceField2dFilter");

  // Initialize node handle
  nodeHandle_ = ros::NodeHandle("~geodesic_distance_filter");

  // Load Parameters
  // Input layer to be processed
  if (!FilterBase<T>::getParam(std::string("input_layer"), inputLayer_)) {
    ROS_ERROR("Geodesic Distance 2D filter did not find parameter `input_layer`.");
    return false;
  }
  ROS_DEBUG("Geodesic Distance 2D filter output_layer = %s.", inputLayer_.c_str());

  // Read output_layers_prefix, to define output grid map layers prefix.
  if (!filters::FilterBase<T>::getParam(std::string("output_layer"), outputLayer_)) {
    ROS_ERROR("Geodesic Distance 2D filter did not find parameter `output_layer`.");
    return false;
  }
  ROS_DEBUG("Geodesic Distance 2D filter output_layer = %s.", outputLayer_.c_str());

  // Read threshold, to define the untraversable areas
  if (!filters::FilterBase<T>::getParam(std::string("threshold"), threshold_)) {
    ROS_ERROR("Geodesic Distance 2D filter did not find parameter `threshold`.");
    return false;
  }
  ROS_DEBUG("Geodesic Distance 2D filter threshold = %f.", threshold_);

  // Option to smooth the field
  // Read flag to binarize defining traversable and non traversable areas
  if (!filters::FilterBase<T>::getParam(std::string("use_field_smoothing"), fieldSmoothing_)) {
    ROS_ERROR("Geodesic Distance 2D filter did not find parameter `use_field_smoothing`.");
    return false;
  }
  ROS_DEBUG("Geodesic Distance 2D filter use_field_smoothing = %s.", (fieldSmoothing_ ? "true" : "false"));

  // Smoothing radius
  if (!filters::FilterBase<T>::getParam(std::string("field_smoothing_radius"), fieldSmoothingRadius_)) {
    ROS_ERROR("Geodesic Distance 2D filter did not find parameter `field_smoothing_radius`.");
    return false;
  }
  ROS_DEBUG("Geodesic Distance 2D filter field_smoothing_radius = %f.", fieldSmoothingRadius_);

  // Attractor subscriber topic
  if (!FilterBase<T>::getParam(std::string("attractor_topic"), attractorTopic_)) {
    ROS_ERROR("Geodesic Distance 2D filter did not find parameter 'attractor_topic'.");
    return false;
  }
  ROS_DEBUG("Geodesic Distance 2D filter attractor_topic = %s.", attractorTopic_.c_str());

  // Read option to normalize gradients
  if (!filters::FilterBase<T>::getParam(std::string("normalize_gradients"), normalizeGradients_)) {
    ROS_ERROR("SDF 2D filter did not find parameter `normalize_gradients`.");
    return false;
  }
  ROS_DEBUG("SDF 2D filter normalize_gradients = %s.", (normalizeGradients_ ? "true" : "false"));

  // Initialize TF listener
  tfListener_ = std::make_shared<tf::TransformListener>();

  // Initialize subscriber
  attractorSubscriber_ = nodeHandle_.subscribe(std::string(attractorTopic_), 1, &GeodesicDistanceField2dFilter::attractorCallback, this);

  // Initialize output layer variables
  obstacleLayer_ = outputLayer_ + "_obstacle_space";
  freeSpaceLayer_ = outputLayer_ + "_free_space";
  gradientXLayer_ = outputLayer_ + "_gradient_x";
  gradientYLayer_ = outputLayer_ + "_gradient_y";
  gradientZLayer_ = outputLayer_ + "_gradient_z";

  return true;
}

template <typename T>
void GeodesicDistanceField2dFilter<T>::attractorCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg) {
  if (mapFrame_ == "not_set") {
    return;
  }

  // Convert attractor goal to grid map frame
  std::string goalFixedFrame = msg->header.frame_id;

  Eigen::Isometry3d eigen_T_fixed_goal = Eigen::Isometry3d::Identity();
  tf::poseMsgToEigen(msg->pose.pose, eigen_T_fixed_goal);

  if (goalFixedFrame != mapFrame_) {
    tf::StampedTransform tf_T_map_goal;

    try {
      tfListener_->lookupTransform(mapFrame_, goalFixedFrame, ros::Time(0), tf_T_map_goal);

      // Convert to Isometry3d
      Eigen::Isometry3d eigen_T_map_fixed = Eigen::Isometry3d::Identity();
      tf::transformTFToEigen(tf_T_map_goal, eigen_T_map_fixed);

      // Update goal
      eigen_T_fixed_goal = eigen_T_map_fixed * eigen_T_fixed_goal;
    } catch (tf::TransformException& ex) {
      ROS_ERROR("%s", ex.what());
      return;
    }
  }

  // If the timestamps are the same, skip (to avoid TF_REPEATED_DATA issue)
  if (msg->header.stamp == attractorStamp_) {
    return;
  }

  // Fill attractor coordinates
  attractorPosition_.x() = eigen_T_fixed_goal.translation().x();
  attractorPosition_.y() = eigen_T_fixed_goal.translation().y();
  attractorStamp_ = msg->header.stamp;

  ROS_DEBUG_STREAM("[GeodesicDistanceField2dFilter] Attractor: [" << attractorPosition_.x() << ", " << attractorPosition_.y() << "]");
}

template <typename T>
bool GeodesicDistanceField2dFilter<T>::update(const T& mapIn, T& mapOut) {
  profiler_ptr_->startEvent("0.update");

  // Copy and fix indexing
  mapOut = mapIn;
  mapOut.convertToDefaultStartIndex();

  // Check if layer exists.
  if (!mapOut.exists(inputLayer_)) {
    ROS_ERROR("Check your threshold types! Type %s does not exist", inputLayer_.c_str());
    return false;
  }

  // Get resolution
  double resolution = mapOut.getResolution();

  if (mapFrame_ == "not_set") {
    // Set standard goal at the center
    attractorPosition_ = mapOut.getPosition();
  }

  // Get frame of elevation map
  mapFrame_ = mapOut.getFrameId();

  // Convert selected layer to OpenCV image
  profiler_ptr_->startEvent("1.preprocess");
  cv::Mat cvLayer;
  const float minValue = mapOut.get(inputLayer_).minCoeffOfFinites();
  const float maxValue = mapOut.get(inputLayer_).maxCoeffOfFinites();
  grid_map::GridMapCvConverter::toImage<float, 1>(mapOut, inputLayer_, CV_32F, minValue, maxValue, cvLayer);
  cv::normalize(cvLayer, cvLayer, minValue, maxValue, cv::NORM_MINMAX);
  cvLayer.convertTo(cvLayer, CV_32F);
  cvLayer *= 255;

  // Apply threshold and compute obstacle masks
  cv::Mat cvFreeSpaceMask = cvLayer;
  cv::Mat cvObstacleSpaceMask;

  // Binarize input layer
  cv::threshold(cvLayer, cvFreeSpaceMask, 255 * threshold_, 255, cv::THRESH_BINARY);
  cvFreeSpaceMask.convertTo(cvFreeSpaceMask, CV_8UC1);
  cv::bitwise_not(cvFreeSpaceMask, cvObstacleSpaceMask);
  cvObstacleSpaceMask.convertTo(cvObstacleSpaceMask, CV_32F);
  cvFreeSpaceMask.convertTo(cvFreeSpaceMask, CV_32F);

  // Preallocate output layer
  cv::Mat cvGeodesicDistance(cvObstacleSpaceMask.size(), cvObstacleSpaceMask.type(), cv::Scalar(0.0));
  cv::Mat maskedGradientsX(cvObstacleSpaceMask.size(), cvObstacleSpaceMask.type(), cv::Scalar(0.0));
  cv::Mat maskedGradientsY(cvObstacleSpaceMask.size(), cvObstacleSpaceMask.type(), cv::Scalar(0.0));
  cv::Mat cvGradientsX(cvObstacleSpaceMask.size(), cvObstacleSpaceMask.type(), cv::Scalar(0.0));
  cv::Mat cvGradientsY(cvObstacleSpaceMask.size(), cvObstacleSpaceMask.type(), cv::Scalar(0.0));
  cv::Mat cvGradientsZ(cvObstacleSpaceMask.size(), cvObstacleSpaceMask.type(), cv::Scalar(0.0));

  // Add free and occupied space as layers
  addMatAsLayer(cvObstacleSpaceMask / 255, obstacleLayer_, mapOut);
  addMatAsLayer(cvFreeSpaceMask / 255, freeSpaceLayer_, mapOut);

  // This little hack makes the fast marching method work
  cvObstacleSpaceMask *= 10;  // This is to enforce the difference between obstacles and free space
  cvObstacleSpaceMask += 1;   // This adds a baseline layer to start the propagation

  // Smooth field by applying Gaussian Smoothing
  // This is similar to the 'saturation' method used in
  // FM2 by Javier V. Gomez: https://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=6582543
  if (fieldSmoothing_) {
    int radiusInPixels = std::max((int)std::ceil(fieldSmoothingRadius_ / mapIn.getResolution()), 3);  // Minimum kernel of size 3
    radiusInPixels = (radiusInPixels % 2 == 0) ? radiusInPixels + 1 : radiusInPixels;

    cv::GaussianBlur(cvObstacleSpaceMask, cvObstacleSpaceMask, cv::Size(radiusInPixels, radiusInPixels), 0);
  }
  profiler_ptr_->endEvent("1.preprocess");

  profiler_ptr_->startEvent("2.gdf_computation");
  // Prepare seeds
  std::vector<cv::Point> seeds;
  grid_map::Index index = getAttractorIndex(mapOut, attractorPosition_);

  // Compute Geodesic field
  seeds.push_back(cv::Point(index.y(), index.x()));  // We need to flip the components because it's an image

  // Compute Geodesic Distance
  GeodesicDistance<float>(cvObstacleSpaceMask, seeds, cvGeodesicDistance);
  profiler_ptr_->endEvent("2.gdf_computation");

  profiler_ptr_->startEvent("3.gdf_gradients");
  // Blur before computing gradients to smooth the image
  cv::GaussianBlur(cvGeodesicDistance, cvGeodesicDistance, cv::Size(5, 5), 0);

  // Compute gradients
  cv::Sobel(cvGeodesicDistance, cvGradientsX, -1, 0, 1, 3, resolution);
  cv::Sobel(cvGeodesicDistance, cvGradientsY, -1, 1, 0, 3, resolution);

  if (normalizeGradients_) {
    // Compute norm
    cv::Mat sqGradientsX;
    cv::Mat sqGradientsY;
    cv::pow(cvGradientsX, 2, sqGradientsX);
    cv::pow(cvGradientsY, 2, sqGradientsY);
    cv::Mat normNormal;
    cv::sqrt(sqGradientsX + sqGradientsY, normNormal);

    // Normalize
    cvGradientsX /= (normNormal + std::numeric_limits<float>::epsilon());
    cvGradientsY /= (normNormal + std::numeric_limits<float>::epsilon());
  }
  profiler_ptr_->endEvent("3.gdf_gradients");

  // Add layers
  profiler_ptr_->startEvent("4.gdf_add_layers");
  addMatAsLayer(cvGeodesicDistance, outputLayer_, mapOut, resolution);
  addMatAsLayer(cvGradientsX, gradientXLayer_, mapOut);
  addMatAsLayer(cvGradientsY, gradientYLayer_, mapOut);
  addMatAsLayer(cvGradientsZ, gradientZLayer_, mapOut);
  profiler_ptr_->endEvent("4.gdf_add_layers");

  mapOut.setBasicLayers({});

  // Timing
  profiler_ptr_->endEvent("0.update");
  ROS_DEBUG_STREAM_THROTTLE(1, "-- Profiler report (throttled (1s)\n" << profiler_ptr_->getReport());

  return true;
}

template <typename T>
grid_map::Index GeodesicDistanceField2dFilter<T>::getAttractorIndex(const T& gridMap, const grid_map::Position& attractorPosition) {
  Position attractor = attractorPosition;

  grid_map::Index attractorIdx;
  grid_map::Index centerIdx;

  gridMap.getIndex(gridMap.getPosition(), centerIdx);
  gridMap.getIndex(attractor, attractorIdx);

  // Check if the attractor is outside the map to snap it to the edge
  if (!gridMap.isInside(attractor)) {
    // Get closest points
    attractor = gridMap.getClosestPositionInMap(attractor);
    // Get index
    gridMap.getIndex(attractor, attractorIdx);
    // Enforce the index to be valid
    attractorIdx.x() = std::max(std::min(attractorIdx.x(), gridMap.getSize().x() - 1), 0);
    attractorIdx.y() = std::max(std::min(attractorIdx.y(), gridMap.getSize().y() - 1), 0);
  }

  // Check if the point is in untraversable areas, so we look for the closest traversable point
  if (gridMap.at(freeSpaceLayer_, attractorIdx) == 0) {
    // // Project a line from the attractor to the center to find the next traversable point
    // const size_t maxCount = 5;
    // size_t count = 0;
    // for (grid_map::LineIterator iterator(gridMap, attractorIdx, centerIdx); !iterator.isPastEnd(); ++iterator) {
    //   if (gridMap.isValid(*iterator, freeSpaceLayer_)) {

    //     grid_map::Index idx = *iterator;
    //     if (gridMap.at(freeSpaceLayer_, idx) > 0) {
    //       count++;
    //     }
    //     if (count > maxCount){
    //       attractorIdx = idx;
    //       break;
    //     }
    //   }
    // }

    double distanceToCenter = (gridMap.getPosition() - attractor).norm();
    for (grid_map::SpiralIterator iterator(gridMap, attractor, distanceToCenter); !iterator.isPastEnd(); ++iterator) {
      if (gridMap.isValid(*iterator, freeSpaceLayer_)) {
        grid_map::Index idx = *iterator;
        if (gridMap.at(freeSpaceLayer_, idx) > 0) {
          attractorIdx = idx;
          // ROS_WARN_STREAM("attractor spiral index: " << index(0) << ", " << index(1));
          break;
        }
      }
    }
  }
  return attractorIdx;
}

template <typename T>
void GeodesicDistanceField2dFilter<T>::addMatAsLayer(const cv::Mat& mat, const std::string& layerName, grid_map::GridMap& gridMap,
                                                     double resolution) {
  // ROS_WARN_STREAM("trying to add layer [" << layerName << "]" << mat.rows << " x " << mat.cols);
  // Get max and min
  double minDistance, maxDistance;
  cv::minMaxLoc(mat, &minDistance, &maxDistance);

  minDistance *= resolution;
  maxDistance *= resolution;

  // cv::namedWindow(layerName + "_original", cv::WINDOW_NORMAL );
  // cv::imshow(layerName + "_original", mat);

  // Normalize sdf to get a greyscale image
  cv::Mat normalized;
  cv::normalize(mat, normalized, 0, 1.0, cv::NORM_MINMAX);
  normalized.convertTo(normalized, CV_32F);

  // Add layer to elevation map
  grid_map::GridMapCvConverter::addLayerFromImage<float, 1>(normalized, layerName, gridMap, minDistance, maxDistance);
}

}  // namespace grid_map

// Explicitly define the specialization for GridMap to have the filter implementation available for testing.
template class grid_map::GeodesicDistanceField2dFilter<grid_map::GridMap>;
// Export the filter.
PLUGINLIB_EXPORT_CLASS(grid_map::GeodesicDistanceField2dFilter<grid_map::GridMap>, filters::FilterBase<grid_map::GridMap>)