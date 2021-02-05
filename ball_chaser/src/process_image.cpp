#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    ROS_INFO_STREAM("[process_image] Making a request to the command_robot svc");
    ball_chaser::DriveToTarget req;
    req.request.angular_z = ang_z;
    req.request.linear_x = lin_x;
    // Make request to the service and pass the velocities to it to drive the robot
    if (!client.call(req))
        ROS_ERROR("Failed to request to the command_robot svc");
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{
    int tier = (int)img.width / 3;
    int mid_tier = tier + tier;
    uint8_t white_pixel = 255;
    // white ball is unknown: (0, 0) will stop the robot
    float lin_x = 0.0;
    float ang_z = 0.0;
    // Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera

    // NOTES:
    //  - row height is ignored (we're not a drone...)
    //  - We're assuming a 3-channel RGB image...
    int col = 0;
    for (int i = 0; i < img.height * img.step; i += 3) {
        col = i % img.width;
        if (img.data[i]     == white_pixel && 
            img.data[i + 1] == white_pixel && 
            img.data[i + 2] == white_pixel) {
                // skid steer: tiny bit of forward motion
                lin_x = 0.05;
                if (col < tier)
                    ang_z = 0.5;    // left
                else if (col >= tier && col < mid_tier)
                    lin_x = 0.5;   // middle --> forward 
                else
                    ang_z = -0.5;   // right
                
                drive_robot(lin_x, ang_z);
                return;
        }
    }
    // ball unknown --> stop
    drive_robot(lin_x, ang_z);
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of making requests to the command_robot service
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}
