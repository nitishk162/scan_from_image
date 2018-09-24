#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include <visualization_msgs/Marker.h>
#include "std_msgs/String.h"
#include <sensor_msgs/LaserScan.h>
#include <stdio.h>
#include <visualization_msgs/Marker.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <cmath>
#include <sensor_msgs/Range.h>
#include <cv.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <map>
#include <math.h>
#include <sensor_msgs/LaserScan.h>
using namespace std;
using namespace cv;

struct points
{
  int x,y;
  
};
uint16_t scan_beams = 628;
float dist_resolution=0.05;
float max_scan_angle = M_PI;
float min_scan_angle = -M_PI;
float scan_resolution= (max_scan_angle - min_scan_angle)/scan_beams ;
float max_invalid_range=1048.0;
float max_range = 30.0;
long int seq_id=0;
std::map<uint16_t, float> scan;
Mat input_image = imread("/home/nitish/catkin_ws/map.pgm",CV_LOAD_IMAGE_GRAYSCALE); 
Mat image = input_image.clone();  
typedef struct points points_t;
void mouse_click_callback();
sensor_msgs::LaserScan get_scan_from_image(int col, int row);
double compute_dist(points_t point1, points_t point2)
{
     double dist=(double)(pow((point1.x-point2.x),2)+pow((point1.y-point2.y),2));


     dist=sqrt(dist);  
     return dist;
}
ros::Publisher scan_pub;
std_msgs::Float32 msg1,msg2;
std::vector<points_t> points;
std::vector<double> distances;

void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
    
     if  ( event == EVENT_LBUTTONDOWN )
     {
          for (uint16_t i = 0; i < scan_beams; ++i)
          {
               scan[i]=max_invalid_range;
          }
          scan_pub.publish(get_scan_from_image(x,y));

     }
}

sensor_msgs::LaserScan get_scan_from_image(int col, int row)
{
     seq_id++;
     points_t pixels;
     float r,theta;
     int pix_max_range=max_range/dist_resolution;
     int start_row, start_col, end_row, end_col;
     int crop_x,crop_y;
     pixels.x=col;
     pixels.y=row;
     points.push_back(pixels);
     start_col =  std::max(0, col - pix_max_range);
     end_col = std::min(image.cols, col + pix_max_range);
     start_row = std::max(0, row - pix_max_range);
     end_row = std::min(image.rows, row + pix_max_range);
     ROS_INFO("ROI:%d,%d,%d,%d",start_col,end_col,start_row,end_row);
     for (uint16_t i = start_col; i < end_col; i++)
     {
          for (uint16_t j = start_row; j < end_row; j++)
          {
               if(image.at<uchar>(j,i)<200)
               {
                   r=sqrt(pow((col - i),2)+pow((row - j),2));
                   r=r*dist_resolution;
                   theta=atan2(col - i, row - j);
                   uint16_t index = (theta - min_scan_angle)/scan_resolution;
                   
                  
                    float width_beam = r * scan_resolution;
                    int skip_checks = dist_resolution/width_beam;
                    for (uint16_t k = index - skip_checks/2; k <= index + skip_checks/2; ++k)
                    {
                         if(r < scan[k] && r < max_range )
                         {
                              scan[k] = r;
                         }
                    }
                                            
               }

          }

     }
     sensor_msgs::LaserScan laser_scan;

     laser_scan.header.seq = seq_id;
     laser_scan.header.stamp = ros::Time::now();
     laser_scan.header.frame_id = "laser";
     laser_scan.angle_min = min_scan_angle;
     laser_scan.angle_max = max_scan_angle;
     laser_scan.angle_increment = (laser_scan.angle_max - laser_scan.angle_min)/scan_beams;
     laser_scan.range_min = 0.0;
     laser_scan.range_max = max_range;
     laser_scan.ranges.resize(scan_beams);
     laser_scan.intensities.resize(scan_beams);
     for (uint16_t i = 0; i < scan_beams; ++i)
     {
          laser_scan.ranges[i] = scan[i];
          laser_scan.intensities[i] = i;
     }

     
     return (laser_scan);
}




int main(int argc, char **argv)
{
     ros::init(argc, argv, "listener");
     ros::NodeHandle nh;
     scan_pub = nh.advertise<sensor_msgs::LaserScan>("/scan",1);

     for (uint16_t i = 0; i < scan_beams; ++i)
     {
          scan[i]=max_invalid_range;
     }

     ros::Rate loop(10);
     int disp_image_size =800;
     Size size(disp_image_size,disp_image_size);//the dst image size,e.g.100x100
     float scale_image = static_cast<float>(input_image.rows)/disp_image_size;
     dist_resolution = scale_image*dist_resolution;
     resize(input_image,input_image,size);//resize image
     image = input_image;
     namedWindow("My Window", 1);
     while(nh.ok())
     {
          mouse_click_callback();
          imshow("My Window", image);
          waitKey(0);

          loop.sleep();
          ros::spinOnce();
     }
     destroyAllWindows();
     return 0;
}



void mouse_click_callback()
{
     
     setMouseCallback("My Window", CallBackFunc, NULL);

}
