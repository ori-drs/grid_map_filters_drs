/*
 * GeodesicFieldFilter.cpp
 *
 *  Computes a geodesic field from a layer, given an attractor
 *  Note: Requires more testing, features can otentially be merged into GeodesicDistanceField2dFilter
 *
 *  Author: Matias Mattamala
 */

#include <grid_map_filters_drs/GeodesicFieldFilter.hpp>

using namespace filters;

namespace grid_map {

template <typename T>
GeodesicFieldFilter<T>::GeodesicFieldFilter() : attractorPosition_(0, 0), attractorStamp_(0), mapFrame_("not_set") {}

template <typename T>
GeodesicFieldFilter<T>::~GeodesicFieldFilter() {}

template <typename T>
bool GeodesicFieldFilter<T>::configure() {
  // Setup profiler
  profiler_ptr_ = std::make_shared<Profiler>("GeodesicFieldFilter");

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

  // Read option to publish path
  if (!filters::FilterBase<T>::getParam(std::string("publish_path"), publishPath_)) {
    ROS_ERROR("SDF 2D filter did not find parameter `publish_path`.");
    return false;
  }
  ROS_DEBUG("SDF 2D filter publish_path = %s.", (publishPath_ ? "true" : "false"));

  // Attractor subscriber topic
  if (!FilterBase<T>::getParam(std::string("path_topic"), pathTopic_)) {
    ROS_ERROR("Geodesic Distance 2D filter did not find parameter 'path_topic'.");
    return false;
  }
  ROS_DEBUG("Geodesic Distance 2D filter path_topic = %s.", pathTopic_.c_str());

  // Initialize TF listener
  tfListener_ = std::make_shared<tf::TransformListener>();

  // Initialize subscriber
  attractorSubscriber_ = nodeHandle_.subscribe(std::string(attractorTopic_), 1, &GeodesicFieldFilter::attractorCallback, this);

  // Initialize path publisher
  if (publishPath_) {
    pathPublisher_ = nodeHandle_.advertise<nav_msgs::Path>(std::string(pathTopic_), 10);
  }

  // Initialize output layer variables
  obstacleLayer_ = outputLayer_ + "_obstacle_space";
  freeSpaceLayer_ = outputLayer_ + "_free_space";
  gradientXLayer_ = outputLayer_ + "_gradient_x";
  gradientYLayer_ = outputLayer_ + "_gradient_y";
  gradientZLayer_ = outputLayer_ + "_gradient_z";

  return true;
}

template <typename T>
void GeodesicFieldFilter<T>::attractorCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg) {
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

  ROS_DEBUG_STREAM("[GeodesicFieldFilter] Attractor: [" << attractorPosition_.x() << ", " << attractorPosition_.y() << "]");
}

template <typename T>
bool GeodesicFieldFilter<T>::update(const T& mapIn, T& mapOut) {
  profiler_ptr_->startEvent("0.update");

  // Copy and fix indexing
  mapOut = mapIn;
  mapOut.convertToDefaultStartIndex();

  // Check if layer exists.
  if (!mapOut.exists(inputLayer_)) {
    ROS_ERROR("Check your layers! Layer %s does not exist", inputLayer_.c_str());
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
  cv::Mat cvLayer;
  const float minValue = mapOut.get(inputLayer_).minCoeffOfFinites();
  const float maxValue = mapOut.get(inputLayer_).maxCoeffOfFinites();
  grid_map::GridMapCvConverter::toImage<float, 1>(mapOut, inputLayer_, CV_32F, minValue, maxValue, cvLayer);
  cv::normalize(cvLayer, cvLayer, minValue, maxValue, cv::NORM_MINMAX);
  cvLayer.convertTo(cvLayer, CV_32F);

  // Preallocate output layer
  cv::Mat cvGeodesicDistance(cvLayer.size(), cvLayer.type(), cv::Scalar(0.0));
  cv::Mat cvGradientsX(cvLayer.size(), cvLayer.type(), cv::Scalar(0.0));
  cv::Mat cvGradientsY(cvLayer.size(), cvLayer.type(), cv::Scalar(0.0));
  cv::Mat cvGradientsZ(cvLayer.size(), cvLayer.type(), cv::Scalar(0.0));

  // Prepare seeds
  std::vector<cv::Point> seeds;
  grid_map::Index index = getAttractorIndex(mapOut, attractorPosition_);

  // Compute Geodesic field
  seeds.push_back(cv::Point(index.y(), index.x()));  // We need to flip the components because it's an image

  // Compute Geodesic Distance
  GeodesicDistance<float>(cvLayer, seeds, cvGeodesicDistance);

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

  // Add layers
  addMatAsLayer(cvGeodesicDistance, outputLayer_, mapOut, resolution);
  addMatAsLayer(cvGradientsX, gradientXLayer_, mapOut);
  addMatAsLayer(cvGradientsY, gradientYLayer_, mapOut);
  addMatAsLayer(cvGradientsZ, gradientZLayer_, mapOut);

  // Fix gradients
  mapOut.at(gradientXLayer_, index) = 0.0;
  mapOut.at(gradientYLayer_, index) = 0.0;
  mapOut.at(gradientZLayer_, index) = 1.0;

  mapOut.setBasicLayers({});

  // Publish path
  if (publishPath_) {
    double stepSize = mapOut.getResolution();  // in meters

    // Integrate the vector field from the starting position
    grid_map::Position pos = mapOut.getPosition();
    // Get gradient
    double dx = mapOut.atPosition(gradientXLayer_, pos);
    double dy = mapOut.atPosition(gradientYLayer_, pos);

    std::string elevationLayer = mapOut.exists("elevation_inpainted") ? "elevation_inpainted" : "elevation";
    double z = mapOut.atPosition(elevationLayer, pos);

    nav_msgs::Path path;
    path.header.frame_id = mapFrame_;
    path.header.stamp = ros::Time::now();

    size_t iter = 0;
    size_t maxIter = 2 * mapOut.getSize()(0);
    while (std::hypot(dx, dy) > 0.1 && iter < maxIter && ros::ok()) {
      pos += grid_map::Position(dx * stepSize, dy * stepSize);
      dx = mapOut.atPosition(gradientXLayer_, pos);
      dy = mapOut.atPosition(gradientYLayer_, pos);

      Index index;
      mapOut.getIndex(pos, index);
      if (mapOut.isValid(index, elevationLayer)) {
        z = mapOut.atPosition(elevationLayer, pos);
      }
      // std::cout << "(" << iter << "/" << maxIter << ") dx: " << dx << ", dy: " << dy << ", pos.x: " << pos.x() << ", pos.y: " << pos.y()
      // << std::endl;

      geometry_msgs::PoseStamped pose;
      pose.pose.position.x = pos.x();
      pose.pose.position.y = pos.y();
      pose.pose.position.z = z + 0.5;
      pose.pose.orientation.w = 1.0;
      path.poses.push_back(pose);
      iter++;
    }
    pathPublisher_.publish(path);
  }

  // Timing
  profiler_ptr_->endEvent("0.update");
  ROS_DEBUG_STREAM_THROTTLE(1, "-- Profiler report (throttled (1s)\n" << profiler_ptr_->getReport());

  return true;
}

template <typename T>
grid_map::Index GeodesicFieldFilter<T>::getAttractorIndex(const T& gridMap, const grid_map::Position& attractorPosition) {
  // Preallocate output index
  grid_map::Index index;

  // Get closest attractor position
  Position closestAttractor = gridMap.getClosestPositionInMap(attractorPosition);

  // Check if the attractor is in traversable areas
  gridMap.getIndex(closestAttractor, index);

  // // Sanity check
  index.x() = std::min(index.x(), gridMap.getSize().x() - 1);
  index.y() = std::min(index.y(), gridMap.getSize().y() - 1);
  index.x() = std::max(index.x(), 0);
  index.y() = std::max(index.y(), 0);

  return index;
}

template <typename T>
void GeodesicFieldFilter<T>::addMatAsLayer(const cv::Mat& mat, const std::string& layerName, grid_map::GridMap& gridMap,
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
template class grid_map::GeodesicFieldFilter<grid_map::GridMap>;
// Export the filter.
PLUGINLIB_EXPORT_CLASS(grid_map::GeodesicFieldFilter<grid_map::GridMap>, filters::FilterBase<grid_map::GridMap>)