/*
 * uTestChessEngine - Class which tests ChessEngine.h
 * <p>
 * This class tests the functionality of ChessEngine.h
 * 
 * 
 * @author Omar Ibrahim
 * @version 0.1 ( Initial development ).
 * @version 1.0 ( Initial release ).
 * 
 * @see ChessEngine.h
*/

    // ////////// //
    // Includes.  //
    // ////////// //
#include <ros/ros.h>

#include "ibo1_irc_api/Chess/ChessEngine.h"
#include "ibo1_irc_api/ProtocolAPI/InternalProtocolDefinition.h"

#include <gtest/gtest.h>


#include <chrono>
#include <thread>



//Testing if chessEngine can play against itself 64 times without an error occuring
TEST(ChessEngine, chessEnginePlayAgainstItself){
    ChessEngine* chessEngine = new ChessEngine("/home/omar/Uni/Major_Project/Interactive-Robot-Chess/DEV/ROS/catkin_ws/src/ibo1_irc_api/data/Chess/stockfish/src/stockfish");
    string chessEngineMove = "";
    BYTE returnedProtocol;
    bool gotError = false;

    for(int i = 0; i < 64; i++){
        
        chessEngine->chessEngineMove(returnedProtocol, chessEngineMove);
        cout << "Returned protocol: " << (int)returnedProtocol << endl;
        if(!gotError){
            if(returnedProtocol != CMD_INTERNAL_CHESSENGINEMOVE) gotError = true;
        }
    }
    delete chessEngine;

    ASSERT_EQ(gotError, false);
}




int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "testChessEngine");
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();

}