#include <ros/ros.h>

#include "ibo1_irc_api/Utility/DataChecker.h"

#include <gtest/gtest.h>



//Testing if correct move format
TEST(DataChecker, correctMoveFormat){
    string move = "e2e1";

    ASSERT_TRUE(DataChecker::isCorrectMoveFormat(move));
}

// Checking if incorrect move format
TEST(DataChecker, inCorrectMoveFormat){
    string move = "h2j1";

    ASSERT_FALSE(DataChecker::isCorrectMoveFormat(move));
}

// Test if correct Promotion format
TEST(DataChecker, correctPromotionFormat){
    string move = "e2e1q";

    ASSERT_TRUE(DataChecker::isCorrectMoveFormatPromotion(move));
}

// Test if incorrect Promotion format
TEST(DataChecker, inCorrectPromotionFormat){
    string move = "e2e1";

    ASSERT_FALSE(DataChecker::isCorrectMoveFormatPromotion(move));
}



int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "tester");
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();

}