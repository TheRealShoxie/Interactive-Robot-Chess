#include <ros/ros.h>

#include "ibo1_IRC_API/Chess/ChessEngine.h"

#include <gtest/gtest.h>


#include <chrono>
#include <thread>




//Testing chessEngine Move return
TEST(Chess, chessEngineCreation){
    ChessEngine* chessEngine = new ChessEngine("/home/omar/Uni/Major_Project/Interactive-Robot-Chess/DEV/ROS/catkin_ws/src/ibo1_IRC_API/data/Chess/stockfish/src/stockfish");
    string chessEngineMove = "";
    string playerMove = "D2D4";
    BYTE returnedProtocol;

    for(int i = 0; i < 180; i++){
        
        chessEngine->chessEngineMove(returnedProtocol, chessEngineMove);
        cout << "________________________________________" << endl;
        cout << "Move: " << chessEngineMove << endl;
        cout << chessEngine->getChessBoardString() << endl;
        cout << chessEngine->getChessBoardFENString() << endl;
        cout << "Turn: " << (int)i << endl;
        cout << "Returned protocol: " << (int)returnedProtocol << endl;
    }
    delete chessEngine;

}



int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "tester");
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();

}