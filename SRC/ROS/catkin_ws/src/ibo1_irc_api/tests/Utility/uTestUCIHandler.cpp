/*
 * uTestUCIHandler- Class which tests UCIHandler.h
 * <p>
 * This class tests the functionality of UCIHandler.h
 * 
 * 
 * @author Omar Ibrahim
 * @version 0.1 ( Initial development ).
 * @version 1.0 ( Initial release ).
 * 
 * @see UCIHandler.h
*/

#include <ros/ros.h>

#include "ibo1_irc_api/Utility/UCIHandler.h"

#include <gtest/gtest.h>




//Testing option extraction with null default value
TEST(UCIHandler, engineOptionExtractNoDefaultValue1){
    EngineOption engineOption;

    EngineOption returnedEngineOption;
    string optionString = "";
    optionString = "option name Debug Log File type string default";
    engineOption.name = "Debug Log File";
    engineOption.typeOfValue = "string";
    engineOption.defaultValue = "";
    engineOption.minValue = "";
    engineOption.maxValue = "";
    engineOption.restValues = "";

    returnedEngineOption = DataCreator::createEngineOption(optionString);
    ASSERT_EQ(engineOption, returnedEngineOption);
}

//Test option with no default value
TEST(UCIHandler, engineOptionExtractNoDefaultValue2){
    EngineOption engineOption;

    EngineOption returnedEngineOption;
    string optionString = "";

    optionString = "option name Clear Hash type button";
    engineOption.name = "Clear Hash";
    engineOption.typeOfValue = "button";
    engineOption.defaultValue = "";
    engineOption.minValue = "";
    engineOption.maxValue = "";
    engineOption.restValues = "";

    returnedEngineOption = DataCreator::createEngineOption(optionString);
    ASSERT_EQ(engineOption, returnedEngineOption);
}

// Test option default, min and max value
TEST(UCIHandler, engineOptionExtractDefaultMinMax1){
    EngineOption engineOption;

    EngineOption returnedEngineOption;
    string optionString = "";

    // For second test
    optionString = "option name Threads type spin default 1 min 1 max 1024";
    engineOption.name = "Threads";
    engineOption.typeOfValue = "spin";
    engineOption.defaultValue = "1";
    engineOption.minValue = "1";
    engineOption.maxValue = "1024";
    engineOption.restValues = "";

    returnedEngineOption = DataCreator::createEngineOption(optionString);
    ASSERT_EQ(engineOption, returnedEngineOption);

}

//Test option with default, mind and max value 2
TEST(UCIHandler, engineOptionExtractDefaultMindMax2){
    EngineOption engineOption;

    EngineOption returnedEngineOption;
    string optionString = "";

    optionString = "option name Hash type spin default 16 min 1 max 33554432";
    engineOption.name = "Hash";
    engineOption.typeOfValue = "spin";
    engineOption.defaultValue = "16";
    engineOption.minValue = "1";
    engineOption.maxValue = "33554432";
    engineOption.restValues = "";

    returnedEngineOption = DataCreator::createEngineOption(optionString);
    ASSERT_EQ(engineOption, returnedEngineOption);
}

//Test option with only default value
TEST(UCIHandler, engineOptionExtractOnlyDefaultValue1){
    EngineOption engineOption;

    EngineOption returnedEngineOption;
    string optionString = "";

    optionString = "option name Ponder type check default false";
    engineOption.name = "Ponder";
    engineOption.typeOfValue = "check";
    engineOption.defaultValue = "false";
    engineOption.minValue = "";
    engineOption.maxValue = "";
    engineOption.restValues = "";

    returnedEngineOption = DataCreator::createEngineOption(optionString);
    ASSERT_EQ(engineOption, returnedEngineOption);
}

//Test option with default value <empty>
TEST(UCIHandler, engineOptionExtractOnlyDefaultValue2){
    EngineOption engineOption;

    EngineOption returnedEngineOption;
    string optionString = "";

    optionString = "option name SyzygyPath type string default <empty>";
    engineOption.name = "SyzygyPath";
    engineOption.typeOfValue = "string";
    engineOption.defaultValue = "<empty>";
    engineOption.minValue = "";
    engineOption.maxValue = "";
    engineOption.restValues = "";

    returnedEngineOption = DataCreator::createEngineOption(optionString);
    ASSERT_EQ(engineOption, returnedEngineOption);
}

//Test option with default value a filepath
TEST(UCIHandler, engineOptionExtractOnlyDefaultValue3){
    EngineOption engineOption;

    EngineOption returnedEngineOption;
    string optionString = "";

    optionString = "option name EvalFile type string default nn-ad9b42354671.nnue";
    engineOption.name = "EvalFile";
    engineOption.typeOfValue = "string";
    engineOption.defaultValue = "nn-ad9b42354671.nnue";
    engineOption.minValue = "";
    engineOption.maxValue = "";
    engineOption.restValues = "";

    returnedEngineOption = DataCreator::createEngineOption(optionString);
    ASSERT_EQ(engineOption, returnedEngineOption);
}

//Test option with default value whitespace
TEST(UCIHandler, engineOptionDefaultValueWhiteSpace){
    EngineOption engineOption;

    EngineOption returnedEngineOption;
    string optionString = "";

    optionString = "option name EvalFile type string default  ";
    engineOption.name = "EvalFile";
    engineOption.typeOfValue = "string";
    engineOption.defaultValue = "";
    engineOption.minValue = "";
    engineOption.maxValue = "";
    engineOption.restValues = " ";

    returnedEngineOption = DataCreator::createEngineOption(optionString);
    ASSERT_EQ(engineOption, returnedEngineOption);
}




int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "testUCIHandler");
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();

}