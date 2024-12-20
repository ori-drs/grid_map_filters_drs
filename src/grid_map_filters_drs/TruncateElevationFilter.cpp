/*
 * TruncateElevationFilter.cpp
 *
 * Truncate elevation values within a range
 *
 *
 */

#include <pluginlib/class_list_macros.h>
#include <Eigen/Dense>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_filters_drs/TruncateElevationFilter.hpp>
#include <stdexcept>
#include <string>

using namespace filters;

namespace grid_map {

template <typename T>
TruncateElevationFilter<T>::TruncateElevationFilter() {}

template <typename T>
TruncateElevationFilter<T>::~TruncateElevationFilter() {}

template <typename T>
bool TruncateElevationFilter<T>::configure() {

    // Load Parameters
    // Input layer to be processed
    if (!FilterBase<T>::getParam(std::string("elevation_layer_name"), elevationLayerName_)) {
        ROS_ERROR("[TruncateElevationFilter] did not find parameter `elevation_layer_name`.");
        return false;
    }
    //ROS_DEBUG("[TruncateElevationFilter] input_layers_prefix = %s.", inputNormalLayersPrefix_.c_str());

    // Base frame
    if (!FilterBase<T>::getParam(std::string("base_frame"), baseFrame_)) {
        ROS_ERROR("[TruncateElevationFilter] did not find parameter 'base_frame'.");
        return false;
    }

    // Map frame
    if (!FilterBase<T>::getParam(std::string("map_frame"), mapFrame_)) {
        ROS_ERROR("[TruncateElevationFilter] did not find parameter 'map_frame'.");
        return false;
    }


    // Get relative upper and lower bound
    if (!FilterBase<T>::getParam(std::string("relative_upper_bound"), upperboundRelative_)) {
        ROS_ERROR("[TruncateElevationFilter] did not find parameter 'relative_upper_bound'.");
        return false;
    }

    if (!FilterBase<T>::getParam(std::string("relative_lower_bound"), lowerboundRelative_)) {
        ROS_ERROR("[TruncateElevationFilter] did not find parameter 'relative_lower_bound'.");
        return false;
    }

    if(upperboundRelative_<0 || lowerboundRelative_<0)
    {
        ROS_ERROR("upper or lower bound must be set to non-negative value");
        return false;
    }

    // Initialize TF listener
    tfListener_ = std::make_shared<tf::TransformListener>();
    
    return true;

}

template <typename T>
bool TruncateElevationFilter<T>::update(const T& mapIn, T& mapOut) {
    // Initialize output layer
    mapOut = mapIn;

    // Check if layer exists.
    if (!mapOut.exists(elevationLayerName_)) {
        ROS_ERROR("[TruncateElevationFilter] Check your layer type! Type %s does not exist", elevationLayerName_.c_str());
        return false;
    }

    // Add filtered layer
    mapOut.add("elevation_truncated", mapOut.get(elevationLayerName_));

    // Get frame of elevation map
    // mapFrame_ = mapOut.getFrameId();

    // Get transformation from target to map
    Eigen::Isometry3d T_map_base = Eigen::Isometry3d::Identity();

    // Recover transformation
    try {
        tf::StampedTransform tf_b2m;
        tfListener_->lookupTransform(mapFrame_, baseFrame_, ros::Time(0), tf_b2m);
        tf::transformTFToEigen(tf_b2m, T_map_base);
    } catch (tf::TransformException& ex) {
        ROS_ERROR("%s", ex.what());
    }

    // Get upper and lower bound 
    double base_elevation = T_map_base.matrix()(2,3); // z-value
    double upperbound = base_elevation + upperboundRelative_;
    double lowerbound = base_elevation - lowerboundRelative_;

    // Filter
    for (grid_map::GridMapIterator iterator(mapOut); !iterator.isPastEnd(); ++iterator) {
        if (mapOut.isValid(*iterator, "elevation_truncated")){
            if(mapOut.at("elevation_truncated", *iterator)<lowerbound || mapOut.at("elevation_truncated", *iterator)>upperbound)
            {
                mapOut.at("elevation_truncated", *iterator) = NAN;
            }

        }
    }

    return true;
}

} // namespace grid_map

// Explicitly define the specialization for GridMap
template class grid_map::TruncateElevationFilter<grid_map::GridMap>;
// Export the filter.
PLUGINLIB_EXPORT_CLASS(grid_map::TruncateElevationFilter<grid_map::GridMap>, filters::FilterBase<grid_map::GridMap>)





























