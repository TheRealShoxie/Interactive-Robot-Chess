#include <ros/ros.h>

#include "std_msgs/String.h"

#include <iostream>

using namespace std;


/*
---------------------------------------------------------------------------------------------------------------------------------
*/

    // ////////// //
    // Callbacks. //
    // ////////// //

// void serverMessageReceived(const MomentArrayStamped msg){
//     returnedMessage = msg;
//     cout << "----------------------------------------------------------" << endl;
//     cout << "Message: " << returnedMessage.moments << endl;
//     cout << "----------------------------------------------------------" << endl;
// }

/*
---------------------------------------------------------------------------------------------------------------------------------
*/

int main (int argc, char **argv){
    ros::init(argc, argv, "cellDetectionNode");
    ros::NodeHandle nh;
    ros:: AsyncSpinner spinner(1);
    spinner.start();

    ros::Publisher cellDetection_pub = nh.advertise<std_msgs::String>("cellDetection_messages", 10);
    //ros::Subscriber server_sub = nh.subscribe("/contour_moments_edge_detection/moments", 10, &serverMessageReceived);

    ros::Rate rate(10);

    while(ros::ok()){


        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}