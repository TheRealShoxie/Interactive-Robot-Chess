#include <ros/ros.h>

#include "ibo1_IRC_API/UCIHandler.h"

#include <gtest/gtest.h>


/*
option name Debug Log File type string default
option name Threads type spin default 1 min 1 max 1024
option name Hash type spin default 16 min 1 max 33554432
option name Clear Hash type button
option name Ponder type check default false
option name MultiPV type spin default 1 min 1 max 500
option name Skill Level type spin default 20 min 0 max 20
option name Move Overhead type spin default 10 min 0 max 5000
option name Slow Mover type spin default 100 min 10 max 1000
option name nodestime type spin default 0 min 0 max 10000
option name UCI_Chess960 type check default false
option name UCI_AnalyseMode type check default false
option name UCI_LimitStrength type check default false
option name UCI_Elo type spin default 1350 min 1350 max 2850
option name UCI_ShowWDL type check default false
option name SyzygyPath type string default <empty>
option name SyzygyProbeDepth type spin default 1 min 1 max 100
option name Syzygy50MoveRule type check default true
option name SyzygyProbeLimit type spin default 7 min 0 max 7
option name Use NNUE type check default true
option name EvalFile type string default nn-ad9b42354671.nnue
*/

//Testing engineOption extractor
TEST(UCIHandler, engineOption){
    UCIHandler uciHandler;
    EngineOption engineOption;

    EngineOption returnedEngineOption;
    string optionString = "";

    // For first test
    optionString = "option name Debug Log File type string default";
    engineOption.name = "Debug Log File";
    engineOption.typeOfValue = "string";
    engineOption.defaultValue = "";
    engineOption.minValue = "";
    engineOption.maxValue = "";
    engineOption.restValues = "";

    returnedEngineOption = uciHandler.createEngineOption(optionString);
    ASSERT_EQ(engineOption, returnedEngineOption);

    // For second test
    optionString = "option name Threads type spin default 1 min 1 max 1024";
    engineOption.name = "Threads";
    engineOption.typeOfValue = "spin";
    engineOption.defaultValue = "1";
    engineOption.minValue = "1";
    engineOption.maxValue = "1024";
    engineOption.restValues = "";

    returnedEngineOption = uciHandler.createEngineOption(optionString);
    ASSERT_EQ(engineOption, returnedEngineOption);

    // For third test
    optionString = "option name Hash type spin default 16 min 1 max 33554432";
    engineOption.name = "Hash";
    engineOption.typeOfValue = "spin";
    engineOption.defaultValue = "16";
    engineOption.minValue = "1";
    engineOption.maxValue = "33554432";
    engineOption.restValues = "";

    returnedEngineOption = uciHandler.createEngineOption(optionString);
    ASSERT_EQ(engineOption, returnedEngineOption);

    // For third test
    optionString = "option name Clear Hash type button";
    engineOption.name = "Clear Hash";
    engineOption.typeOfValue = "button";
    engineOption.defaultValue = "";
    engineOption.minValue = "";
    engineOption.maxValue = "";
    engineOption.restValues = "";

    returnedEngineOption = uciHandler.createEngineOption(optionString);
    ASSERT_EQ(engineOption, returnedEngineOption);
}



int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "tester");
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();

}