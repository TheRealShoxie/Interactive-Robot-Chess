#include <ros/ros.h>

#include "ibo1_IRC_API/Utility/DataChecker.h"

#include <gtest/gtest.h>



//Testing engineOption extractor
TEST(DataChecker, correctMove){
    string move = "e2e1";

    ASSERT_TRUE(DataChecker::isCorrectMoveFormat(move));
}

TEST(DataChecker, correctPromotion){
    string move = "e2e1q";

    ASSERT_TRUE(DataChecker::isCorrectMoveFormatPromotion(move));
}



int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "tester");
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();

}