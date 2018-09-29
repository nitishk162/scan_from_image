#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/LaserScan.h>
#include <stdio.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <cmath>
#include <cv.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <map>
#include <math.h>
#include <ros/package.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


using namespace cv;
//TODO: add check if particular beam is present or not within scan angle

int scan_beams;
double dist_resolution, fov, scan_resolution, max_invalid_range, max_range;
std::string frame_id;
bool test_with_mouse_click;
long int seq_id = 0;
std::map<uint16_t, float> scan;
Mat input_image,image;
void mouse_click_callback();
sensor_msgs::LaserScan get_scan_from_image(int col, int row);
ros::Publisher scan_pub;
std::vector<double> distances;
geometry_msgs::TransformStamped image_to_map;
double get_theta_image(int col_a, int row_a, int col_b, int row_b)
{
    // row_a = -1 * row_a;// representing row as a true Y axis 
    // row_b = -1 * row_b;
    double angle = atan2(row_b - row_a, col_b - col_a);
    return angle;

}

geometry_msgs::TransformStamped get_transform(std::vector<double> map_origin, Mat &image)
{
    geometry_msgs::Pose pose_msg;
    pose_msg.position.x = map_origin[0];
    pose_msg.position.y = map_origin[1];
    pose_msg.position.z = 0;
    tf2::Quaternion q;
    q.setRPY(0, 0, map_origin[2]);
    pose_msg.orientation = tf2::toMsg(q);
    tf2::Transform map_image_origin_to_map, map_to_map_image_origin, image_to_map_image_origin;
    tf2::fromMsg(pose_msg, map_to_map_image_origin);
    map_image_origin_to_map = map_to_map_image_origin.inverse();
    pose_msg.position.x = 0;
    pose_msg.position.y = -1 * image.rows * dist_resolution;
    pose_msg.position.z = 0;
    q.setRPY(0, 0, 0);
    pose_msg.orientation = tf2::toMsg(q);
    tf2::fromMsg(pose_msg, image_to_map_image_origin);
    tf2::Transform image_to_map_origin;
    image_to_map_origin = image_to_map_image_origin*map_image_origin_to_map;
    image_to_map.transform = tf2::toMsg(image_to_map_origin);
    image_to_map.header.seq = 1;
    image_to_map.header.stamp = ros::Time::now();
    image_to_map.header.frame_id = 'image';
    image_to_map.child_frame_id = 'map';
    ROS_INFO("transform:%f, %f,",image_to_map.transform.translation.x, image_to_map.transform.translation.y);

    return image_to_map;
}

void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
    
    if  ( event == EVENT_LBUTTONDOWN )
    {
        for (uint16_t i = 0; i < scan_beams; ++i)
        {
            scan[i] = max_invalid_range;
        }
        scan_pub.publish(get_scan_from_image(x,y));

    }

}

sensor_msgs::LaserScan get_scan_from_image(int col, int row)
{
    seq_id++;
    sensor_msgs::LaserScan laser_scan;
    laser_scan.header.seq = seq_id;
    laser_scan.header.stamp = ros::Time::now();
    laser_scan.header.frame_id = frame_id;
    laser_scan.angle_min = -fov/2;
    laser_scan.angle_max = fov/2;
    laser_scan.angle_increment = (laser_scan.angle_max - laser_scan.angle_min)/scan_beams;
    laser_scan.range_min = 0.0;
    laser_scan.range_max = max_range;
    laser_scan.ranges.resize(scan_beams);
    laser_scan.intensities.resize(scan_beams);
    float r,theta;
    int pix_max_range = max_range/dist_resolution;
    int start_row, start_col, end_row, end_col;
    int crop_x,crop_y;
    start_col =  std::max(0, col - pix_max_range);
    end_col = std::min(image.cols, col + pix_max_range);
    start_row = std::max(0, row - pix_max_range);
    end_row = std::min(image.rows, row + pix_max_range);
    ROS_INFO("ROI:%d,%d,%d,%d   ,%f", start_col, end_col, start_row, end_row, dist_resolution);
    for (uint16_t i = start_col; i < end_col; i++)
    {
        for (uint16_t j = start_row; j < end_row; j++)
        {
            if(image.at<uchar>(j,i)<200)
            {
                r = sqrt(pow((col - i),2)+pow((row - j),2));
                r = r*dist_resolution;
                theta = get_theta_image(i, j, col, row);
                int index = (theta - laser_scan.angle_min)/scan_resolution;
                if (index < 0 || index >=scan_beams)
                    continue;

                float width_beam = r * scan_resolution;
                int skip_checks = dist_resolution/width_beam;
                for (uint16_t k = std::max(0,index - skip_checks/2); k <= std::min(index + skip_checks/2, scan_beams); ++k)
                {
                    if(r < scan[k] && r < max_range )
                    {
                        scan[k] = r;
                    }
                }

            }

        }

    }

    for (uint16_t i = 0; i < scan_beams; ++i)
    {
        laser_scan.ranges[i] = scan[i];
        laser_scan.intensities[i] = i;
    }


    return (laser_scan);
}

void pose_callback(const geometry_msgs::PoseStamped msg)
{
    ROS_INFO("pixel:%f ,%f",msg.pose.position.x,msg.pose.position.y);
    geometry_msgs::PoseStamped image_pose;
    tf2::doTransform(msg, image_pose, image_to_map);
    int col_pix = image_pose.pose.position.x / dist_resolution;
    int row_pix = -1 * image_pose.pose.position.y / dist_resolution;// row corresponds to negative y axis
    ROS_INFO("pixel:%d ,%d",col_pix,row_pix);
    ROS_INFO("pixel:%f ,%f",image_pose.pose.position.x,image_pose.pose.position.x);
    scan_pub.publish(get_scan_from_image(col_pix,row_pix));

}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener");
    ros::NodeHandle nh;
    std::string map_name;
    scan_pub = nh.advertise<sensor_msgs::LaserScan>("/scan",1);
    ros::Subscriber pose_sub = nh.subscribe("/pose_stamped", 1, &pose_callback);
    //Loading laser scan information
    nh.param("/scan_params/fov", fov, 2*M_PI );
    nh.param("/scan_params/max_range", max_range, 30.0);
    nh.param("/scan_params/max_invalid_range", max_invalid_range, 1048.0);
    nh.param("/scan_params/scan_beams", scan_beams, 628);
    nh.param("/scan_params/test_with_mouse_click", test_with_mouse_click, true);
    nh.param("/scan_params/frame_id", frame_id, std::string("laser"));
    scan_resolution= (fov)/scan_beams ;
    std::vector<double> origin;
    if(!nh.getParam("/map_data/origin",origin))
    {
        ROS_ERROR("cant find the parameters of the map");
        return 0;
    }
    if (!nh.getParam("/map_data/resolution",dist_resolution))
    {
        ROS_ERROR("cant find the parameters of the map");
        return 0;
    }
    if (!nh.getParam("/map_data/image",map_name))
    {
        ROS_ERROR("cant find the parameters of the map");
        return 0;
    }

    std::string image_location = ros::package::getPath("scan_from_image");
    image_location = image_location + "/launch/"+map_name;
    ROS_INFO("image_location is:%s", image_location.c_str());
    input_image = imread(image_location.c_str(), CV_LOAD_IMAGE_GRAYSCALE);
    image = input_image.clone();
     ROS_INFO("transform:%f, %f,",origin[0], origin[1]);
    image_to_map = get_transform(origin, input_image);

    for (uint16_t i = 0; i < scan_beams; ++i)
    {
        scan[i]=max_invalid_range;
    }


    ros::Rate loop(10);
    if(test_with_mouse_click)
    {    
        int disp_image_size = 800;
        Size size(disp_image_size,disp_image_size);//the dst image size,e.g.100x100
        float scale_image = static_cast<float>(input_image.rows)/disp_image_size;
        dist_resolution = scale_image*dist_resolution;
        resize(input_image, input_image,size);//resize image
        namedWindow("My Window", 1);
    }    
    image_to_map = get_transform(origin, input_image);
    image = input_image;
    
    while(nh.ok())
    {
        if (test_with_mouse_click)
        {
            mouse_click_callback();
            imshow("My Window", image);
            waitKey(0);
        }
        

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
