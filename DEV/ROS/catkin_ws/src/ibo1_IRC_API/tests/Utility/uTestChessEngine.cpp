#include <ros/ros.h>

#include "ibo1_IRC_API/Chess/ChessEngine.h"

#include <gtest/gtest.h>



//Testing engineOption extractor
TEST(Chess, chessEngineCreation){

}



int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "tester");
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();

}