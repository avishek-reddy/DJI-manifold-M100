// standard directories
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h> //obstacle distance & ultrasonic
#include "std_msgs/Float32MultiArray.h"

// DJIOSDK includes
#include "DJI_guidance.h"
#include "DJI_utility.h"

//ros::Publisher altitude_pub;
float current_ultrasonic[5];
float ultrasonic_last_values[5];


class UltrasonicFilter
{
    public:
        float ranges[5];
        float intensities[5];

        void callback(const sensor_msgs::LaserScan::ConstPtr& msg);
};

void UltrasonicFilter::callback (const sensor_msgs::LaserScan::ConstPtr& msg)
{
    for(int i = 0; i < msg->ranges.size(); i++)
    {
        ranges[i] = msg->ranges[i];
        intensities[i] = msg->intensities[i];
    }
}


int main(int argc, char** argv)
{

    //initialize ROS
    ros::init(argc, argv, "guidanceUltrasonicFilterNode");
    ros::NodeHandle my_node;

    UltrasonicFilter ultrasonicFilter; 

    ros::Subscriber ultrasonicSub = my_node.subscribe("guidance/ultrasonic" , 1, &UltrasonicFilter::callback, &ultrasonicFilter);
    ros::Publisher ultrasonicModPub = my_node.advertise<std_msgs::Float32MultiArray>("ultrasonic_new", 100);

    ROS_INFO("%f", ultrasonicFilter.ranges[4]);

    ros::Rate loop_rate(50);	
	
    while(ros::ok())
    {
        std_msgs::Float32MultiArray array;
        array.data.clear();
        for(int i = 0; i < 5; i++)
        {
            if(ultrasonicFilter.intensities[i] == 1 || ultrasonicFilter.ranges[i] > 0)
            {
                current_ultrasonic[i] = ultrasonicFilter.ranges[i];
            }
            else
            {   
                current_ultrasonic[i] = ultrasonic_last_values[i];
            }
            
            ultrasonic_last_values[i] = current_ultrasonic[i];
            
            array.data.push_back(current_ultrasonic[i]);
        }
        

        ultrasonicModPub.publish(array);
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    
    ros::spin();
    return 0;
}
