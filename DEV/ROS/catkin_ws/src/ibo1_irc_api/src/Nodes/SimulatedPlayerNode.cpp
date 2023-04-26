#include <ros/ros.h>



//Ros msg type includes
#include <gazebo/SetModelState.h>


/*
---------------------------------------------------------------------------------------------------------------------------------
*/
    // ////////////////// //
    //  Global variables. //
    // ////////////////// //


/*
---------------------------------------------------------------------------------------------------------------------------------
*/

    // ////////// //
    // Callbacks. //
    // ////////// //



/*
---------------------------------------------------------------------------------------------------------------------------------
*/

    // ////////// //
    // Methods.   //
    // ////////// //



/*
---------------------------------------------------------------------------------------------------------------------------------
*/

int main (int argc, char **argv){
    ros::init(argc, argv, "simulatedPlayerNode");
    ros::NodeHandle nh;
    ros:: AsyncSpinner spinner(1);
    spinner.start();

    ros::ServiceClient gazeboModelSet_client = nh.serviceClient<gazebo::SetModelState>("/gazebo/set_model_state");


    ros::Rate rate(10);

    while(ros::ok()){


        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}