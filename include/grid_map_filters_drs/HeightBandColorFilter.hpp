/*
 * HeightBandColorFilter.hpp
 *
 * Basic idea: given the base frame and map frame
 * Find the robot position in the map frame
 * For elevation within a boundary of the robot base height, normalize to 0~1 and will be used as color layer
 * Otherwise, set to 0 if lower than lowerbound or 1 if higher than upperbound
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
 * Set color band within a height range
 */
template <typename T>
class HeightBandColorFilter : public filters::FilterBase<T> {
public:

    /*!
    * Constructor
    */
    HeightBandColorFilter();

    /*!
    * Destructor.
    */
    virtual ~HeightBandColorFilter();

    /*!
    * Configures the filter from parameters on the parameter server.
    */
    virtual bool configure();
    

    /*!
   * Check if the elevation is within the range and set to normalized value, otherwise set to 0 or 1
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


} // end of namespace grid_map