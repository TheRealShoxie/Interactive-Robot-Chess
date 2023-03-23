#include <ros/ros.h>

#include <std_msgs/String.h>



int main (int argc, char **argv){
    ros::init(argc, argv, "chessWrapper");
    ros::NodeHandle nh;

    ros::Publisher chessWrapper_pub = nh.advertise<std_msgs::String>("/chessWrapper_messages", 1);

    

    return 0;

    ros::Rate rate(1);
    while(ros::ok()){



        rate.sleep();
    }


    return 0;
}