/*
 * uTestDataManipulation - Class which tests DataManipulation.h
 * <p>
 * This class tests the functionality of DataManipulation.h
 * 
 * 
 * @author Omar Ibrahim
 * @version 0.1 ( Initial development ).
 * @version 1.0 ( Initial release ).
 * 
 * @see DataManipulation.h
*/

#include <ros/ros.h>

#include "ibo1_irc_api/Utility/DataManipulation.h"

#include <gtest/gtest.h>



int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "tester");
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();

}