
#include <ros/ros.h>

#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_cv/grid_map_cv.hpp>


#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

using namespace grid_map;

class LidarElevationMapFilter
{
   public:
   LidarElevationMapFilter(ros::NodeHandle& nh);
   
   void gridMapCallback(const grid_map_msgs::GridMap& msg);
   
   private:
   ros::NodeHandle nh_;
   int erosion_size_;
   std::string input_topic_;
   std::string output_topic_;
   ros::Publisher filtered_map_pub_;
   ros::Subscriber elevation_map_sub_;
};

LidarElevationMapFilter::LidarElevationMapFilter(ros::NodeHandle& nh): 
                                                      nh_(nh)
{
  
  nh_.param("erosion_size", erosion_size_, 2);
  nh_.param("input_topic", input_topic_, std::string("/elevation_mapping/elevation_map_raw"));
  nh_.param("output_topic", output_topic_, std::string("/elevation_mapping/elevation_map_filtered"));

  elevation_map_sub_ = nh_.subscribe(input_topic_, 1, &LidarElevationMapFilter::gridMapCallback, this);

  filtered_map_pub_ = nh_.advertise<grid_map_msgs::GridMap>(output_topic_, 1, true);

}

void LidarElevationMapFilter::gridMapCallback(const grid_map_msgs::GridMap& msg) {
  GridMap input_map;
  GridMapRosConverter::fromMessage(msg, input_map);

  // Apply filter chain.
  /*grid_map::GridMap output_map;
  if (!filter_chain_.update(input_map, output_map)) {
    ROS_ERROR("Could not update the grid map filter chain!");
    return;
  }*/

  grid_map::Matrix& data = input_map["traversability"];
  for (GridMapIterator iterator(input_map); !iterator.isPastEnd(); ++iterator)
  {
    const Index index(*iterator);
    // everyting that is not seen ( Nan ) is traversable
    if (!std::isfinite(data(index(0), index(1))))
    {
      data(index(0), index(1)) = 1.0;
    }
  }


  // Erode the traversable image
  cv::Mat original_image, erode_image;
  GridMapCvConverter::toImage<unsigned short, 1>(input_map, "traversability", CV_16UC1, 0.0, 0.5, original_image);

  int erode_size =  erosion_size_;
  cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE,
                                       cv::Size(2*erode_size + 1, 2*erode_size + 1),
                                       cv::Point(erode_size, erode_size));

  erode(original_image, erode_image, element);

  GridMapCvConverter::addLayerFromImage<unsigned short, 1>(erode_image, "traversability_dilated", input_map, 0.0, 1.0);

  // Publish filtered output grid map.
  grid_map_msgs::GridMap output_msg;
  GridMapRosConverter::toMessage(input_map, output_msg);
  filtered_map_pub_.publish(output_msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "grid_map_erosion");
  ros::NodeHandle nodeHandle("~");
  LidarElevationMapFilter LidarElevationMapFilter(nodeHandle);

  ros::spin();

  return 0;
}
