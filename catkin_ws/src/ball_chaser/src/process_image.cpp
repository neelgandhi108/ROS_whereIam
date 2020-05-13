#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>
#include <numeric>

using namespace std;
ros::ServiceClient client;

void drive_robot(float lin_x, float ang_z)
{
    
    ROS_INFO_STREAM("Driving the robot to the target.");

    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    if (!client.call(srv)) {
	    ROS_ERROR("Failed to call service DriveToTarget.");
	}
}

void process_image_callback(const sensor_msgs::Image img)
{

    int white_pixel = 255;

    int height = img.height;
    int step = img.step;

    float x = 0.0;
    float z = 0.0;
  
    float offset_accumulated = 0;
    int count_total = 0;

  
    for (int i = 0; i < height ; i++) {
        for (int j = 0; j < step; j++) {
            if (img.data[i * step + j] == white_pixel) {
                // vec_cout[j]++;
                offset_accumulated += j - step / 2.0;
                count_total++;
            }
        }
    }

    if (count_total == 0) {
        x = 0.0;
        z = 0.0;
    }
    else {
        x = 0.1;
       
        z = -4.0 * offset_accumulated / count_total / (step /2.0);
    }
    
  
    drive_robot(x, z);
}

int main(int argc, char** argv)
{
   
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);
    ros::spin();

    return 0;
}
