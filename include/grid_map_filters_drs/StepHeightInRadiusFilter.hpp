/*
 * StepHeightInRadiusFilter.hpp
 *
 * Basic idea: step height of a cell is the *largest* difference between the center point  
 * and its neighbors in radius
 * 
 * *largest* means selecting the max value, we can also use other method to calculare (e.g., mean)
 */

#pragma once

#include <string>
#include <ros/ros.h>
#include <filters/filter_base.hpp>
#include <grid_map_core/GridMap.hpp>
#include <cmath>
#include <algorithm>
#include <grid_map_core/grid_map_core.hpp>

#include <pluginlib/class_list_macros.h>

namespace grid_map {

/*!
 * Filter class to find the step height inside a radius.
 */
template <typename T>
class StepHeightInRadiusFilter : public filters::FilterBase<T> {
public:
    /*!
    * Constructor
    */
    StepHeightInRadiusFilter();

    /*!
    * Destructor.
    */
    virtual ~StepHeightInRadiusFilter();

    /*!
    * Configures the filter from parameters on the Parameter Server
    */
    virtual bool configure();    

    /*!
    * Computes for each value in the input layer the step height in a radius around it
    * Saves result to the output layer
    * @param mapIn grid map containing the input layer.
    * @param mapOut grid map containing the layers of the input map and the new layer.
    */
    virtual bool update(const T& mapIn, T& mapOut);

private:
    //! Radius to take the mean from.
    double radius_;

    //! Input layer name.
    std::string inputLayer_;

    //! Output layer name.
    std::string outputLayer_;

    // ! Method
    std::string method_;
};


} // namespace grid_map