/*
 * StepHeightInRadiusFilter.hpp
 *
 * Basic idea: step height of a cell is the *largest* difference between the center point  
 * and its neighbors in radius
 * 
 * *largest* means selecting the max value, we can also use other method to calculare (e.g., mean)
 */



#include <grid_map_filters_drs/StepHeightInRadiusFilter.hpp>



using namespace filters;


namespace grid_map {

template <typename T>
StepHeightInRadiusFilter<T>::StepHeightInRadiusFilter() : radius_(0.0) {}

template <typename T>
StepHeightInRadiusFilter<T>::~StepHeightInRadiusFilter(){};


template <typename T>
bool StepHeightInRadiusFilter<T>::configure() {
    if (!FilterBase<T>::getParam(std::string("radius"), radius_)) {
        ROS_ERROR("StepHeightInRadius filter did not find parameter `radius`.");
        return false;
    }

    if (radius_ < 0.0) {
        ROS_ERROR("StepHeightInRadius filter: Radius must be greater than zero.");
        return false;
    }

    ROS_DEBUG("Radius = %f.", radius_);

    if (!FilterBase<T>::getParam(std::string("input_layer"), inputLayer_)) {
        ROS_ERROR("StepHeightInRadius filter did not find parameter `input_layer`.");
        return false;
    }

    ROS_DEBUG("StepHeightInRadius input layer is = %s.", inputLayer_.c_str());

    if (!FilterBase<T>::getParam(std::string("output_layer"), outputLayer_)) {
        ROS_ERROR("StepHeightInRadius filter did not find parameter `output_layer`.");
        return false;
    }


    if (!FilterBase<T>::getParam(std::string("method"), method_)) {
        ROS_ERROR("StepHeightInRadius filter did not find parameter `method`.");
        return false;
    }

    ROS_DEBUG("StepHeightInRadius output_layer = %s.", outputLayer_.c_str());
    return true;
}


template <typename T>
bool StepHeightInRadiusFilter<T>::update(const T& mapIn, T& mapOut) {
    // Add new layers to the elevation map.
    mapOut = mapIn;
    mapOut.add(outputLayer_);

    double value{NAN};

    // First iteration through the elevation map.
    for (grid_map::GridMapIterator iterator(mapIn); !iterator.isPastEnd(); ++iterator) {
        //double valueSum = 0.0;
        double value_curr = mapIn.at(inputLayer_, *iterator);

        std::vector<double> vec_value;
        int counter = 0;
        // Requested position (center) of circle in map.
        Eigen::Vector2d center;
        mapIn.getPosition(*iterator, center);

        // Find the mean in a circle around the center
        for (grid_map::CircleIterator submapIterator(mapIn, center, radius_); !submapIterator.isPastEnd(); ++submapIterator) {
        if (!mapIn.isValid(*submapIterator, inputLayer_)) {
            continue;
        }
        value = std::abs(mapIn.at(inputLayer_, *submapIterator) - value_curr);
        // valueSum += value;
        vec_value.push_back(value);
        counter++;
        }

        if (counter != 0) {
            if(method_=="max")
            {
                float temp = *std::max_element(vec_value.begin(),vec_value.end());

                mapOut.atPosition(outputLayer_, center) = temp;
            }
            else if(method_=="mean")
            {
                double avg=0;
                for(int i=0;i<vec_value.size();i++)
                {
                    avg = avg + vec_value[i];
                }
                avg = avg / vec_value.size();
                mapOut.atPosition(outputLayer_, center) = avg;
            }
            else
            {
                std::cout << "Not valid method! EXIT!" << std::endl;
                exit(EXIT_FAILURE);
            }
        }
    }

    return true;
}


} // namespace grid_map

// Explicitly define the specialization for GridMap
template class grid_map::StepHeightInRadiusFilter<grid_map::GridMap>;
// Export the filter.
PLUGINLIB_EXPORT_CLASS(grid_map::StepHeightInRadiusFilter<grid_map::GridMap>, filters::FilterBase<grid_map::GridMap>)