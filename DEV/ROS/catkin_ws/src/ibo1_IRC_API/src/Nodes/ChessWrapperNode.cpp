#include <ros/ros.h>

#include <ibo1_IRC_API/Utility/FileHandler.h>
#include <ibo1_IRC_API/Server/IRCServer.h>

#include <std_msgs/String.h>
#include <ibo1_IRC_API/Protocol.h>


static string chessEnginesFilePathName = "/home/omar/Uni/Major_Project/Interactive-Robot-Chess/DEV/ROS/catkin_ws/src/ibo1_IRC_API/data/Chess/chessEngines.txt";
static ibo1_IRC_API::Protocol returnedProtocol;
static ChessEngine *chessEnginePointer;

/*
---------------------------------------------------------------------------------------------------------------------------------
*/

    // ////////// //
    // Callbacks. //
    // ////////// //

void serverMessageReceived(const ibo1_IRC_API::Protocol& msg){
    returnedProtocol = msg;
    cout << "I received following on chessWrapper from server: " << endl;
    cout << "CmdByte: " << (int)returnedProtocol.cmd << endl;
}


/*
---------------------------------------------------------------------------------------------------------------------------------
*/

    // //////////////////// //
    // Internal Functions.  //
    // //////////////////// //


void wrapperLogic(vector<ChessEngineDefinitionStruct>& chessEngines, ros::Publisher *chessWrapper_pub){
    vector<BYTE> response;
    ibo1_IRC_API::Protocol sendProtocol;

    BYTE gotCMD = returnedProtocol.cmd;

    switch (gotCMD)
    {
        //Get chess engine names
        case CMD_GETCHESSENGINES:{
            string chessEngineNames = "";
            for(ChessEngineDefinitionStruct ce : chessEngines){
                chessEngineNames += ce.name;
                chessEngineNames += "\u241f";
            }
            copy(chessEngineNames.begin(), chessEngineNames.end(), std::back_inserter(response));
            sendProtocol.cmd = CMD_GETCHESSENGINES;
            sendProtocol.data = response;
            chessWrapper_pub->publish(sendProtocol);

            break;
        }
        case CMD_STARTCHESSENGINE:{
            string toStartChessEngineName = "";
            string toStartChessEngineFilePathName;
            DataCreator::convertBytesToString(returnedProtocol.data, toStartChessEngineName);

            cout << "ChessEngine Name to start: " << toStartChessEngineName << endl;

            for(ChessEngineDefinitionStruct ce : chessEngines){
                if(ce.name.compare(toStartChessEngineName) == 0){
                    toStartChessEngineFilePathName = ce.filePathName;
                }
            }

            cout << toStartChessEngineFilePathName << endl;

            if(toStartChessEngineFilePathName.empty()){
                sendProtocol.cmd = ERROR_CMD_CHESSENGINEDOESNTEXIST;
                sendProtocol.data = response;
                cout << "Couldnt find Chess Engine Name!" << endl;
            }else{
                ChessEngine currentChessEngine(toStartChessEngineFilePathName);

                cout << chessEnginePointer << endl;

                if(!(chessEnginePointer == 0)){
                    chessEnginePointer->closeEngine();
                }

                chessEnginePointer = &currentChessEngine;
                sendProtocol.cmd = CMD_STARTCHESSENGINE;
                cout << "Started Chess Engine!" << endl;
            }

            sendProtocol.data = response;
            chessWrapper_pub->publish(sendProtocol);
            break;
        }

        case CMD_STOPCHESSENGINE:{
            if(chessEnginePointer == 0){
                sendProtocol.cmd == ERROR_CMD_NOCHESSENGINERUNNING;
            }else{
                chessEnginePointer->closeEngine();
                chessEnginePointer = 0;
                sendProtocol.cmd == CMD_STOPCHESSENGINE;
            }
            sendProtocol.data = response;
            chessWrapper_pub->publish(sendProtocol);
            break;
        }

        default:
            break;
    }

    returnedProtocol.cmd = (BYTE)0x00;

}


/*
---------------------------------------------------------------------------------------------------------------------------------
*/

int main (int argc, char **argv){
    ros::init(argc, argv, "chessWrapper");
    ros::NodeHandle nh;
    ros:: AsyncSpinner spinner(1);
    spinner.start();

    ros::Publisher chessWrapper_pub = nh.advertise<ibo1_IRC_API::Protocol>("chessWrapper_messages", 10);
    ros::Subscriber server_sub = nh.subscribe("/ircServer_messages", 10, &serverMessageReceived);

    vector<ChessEngineDefinitionStruct> chessEngines = FileHandler::readChessEngines(chessEnginesFilePathName);


    ros::Rate rate(10);

    //Waiting till ros system subscribers are activated
    while(chessWrapper_pub.getNumSubscribers() == 0){
        rate.sleep();
    }

    while(ros::ok()){

        

        wrapperLogic(chessEngines, &chessWrapper_pub);



        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}