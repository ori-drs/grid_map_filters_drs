/*
 * TruncateElevationFilter.hpp
 *
 * Basic idea: given the base frame and map frame
 * Find the robot position in the map frame
 * For elevation within a boundary of the robot base height, preserve
 * Otherwise, set to nan
 * 
 *
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
 * Truncate elevation values within a range
 */
template <typename T>
class TruncateElevationFilter : public filters::FilterBase<T> {
public:

    /*!
    * Constructor
    */
    TruncateElevationFilter();

    /*!
    * Destructor.
    */
    virtual ~TruncateElevationFilter();

    /*!
    * Configures the filter from parameters on the parameter server.
    */
    virtual bool configure();
    

    /*!
   * Check if the elevation is within the range, otherwise set to nan
   * @param mapIn GridMap with the different layers to apply a threshold.
   * @param mapOut GridMap with the threshold applied to the layers.
   */
    virtual bool update(const T& mapIn, T& mapOut);

private:
    // Elevation Layer Name
    std::string elevationLayerName_;

    // Base frame
    std::string baseFrame_;
    // Map frame
    std::string mapFrame_;

    // TF listener
    std::shared_ptr<tf::TransformListener> tfListener_;

    // relative upper and lower bound (positive)
    double upperboundRelative_;
    double lowerboundRelative_;


};

} // namespace grid_map














