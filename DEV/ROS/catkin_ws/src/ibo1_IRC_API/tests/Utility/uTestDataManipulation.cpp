#include <ros/ros.h>

#include "ibo1_IRC_API/Utility/DataManipulation.h"

#include <gtest/gtest.h>



//Testing engineOption extractor
TEST(DataManipulation, subStringExtraction){
    string stringToExtract = "";
    string returned = "";
    string splitter = "";
    returned = DataManipulation::subString(stringToExtract,returned);

    ASSERT_EQ(returned, "");
}



int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "tester");
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();

}