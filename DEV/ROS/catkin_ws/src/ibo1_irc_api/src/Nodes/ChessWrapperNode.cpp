#include <ros/ros.h>

#include <ibo1_irc_api/Utility/FileHandler.h>

#include <ibo1_irc_api/Protocol.h>

/*
---------------------------------------------------------------------------------------------------------------------------------
*/
    // ////////////////// //
    //  Global variables. //
    // ////////////////// //
    
static string chessEnginesFilePathName = "/home/omar/Uni/Major_Project/Interactive-Robot-Chess/DEV/ROS/catkin_ws/src/ibo1_irc_api/data/Chess/chessEngines.txt";

// For ros subscribers & publisher
ibo1_irc_api::Protocol returnedProtocol;
ros::Publisher* systemStateMachine_pub_ptr;

// For ChessEngine
ChessEngine *chessEnginePointer;

/*
---------------------------------------------------------------------------------------------------------------------------------
*/

    // ////////// //
    // Callbacks. //
    // ////////// //

void chessWrapperMessageReceived(const ibo1_irc_api::Protocol& msg){
    returnedProtocol = msg;
    cout << "----------------------------------------------------------" << endl;
    cout << "I received following on chessWrapper: " << endl;
    cout << "I received from: " << (int)returnedProtocol.sender << endl;
    cout << "CmdByte: " << (int)returnedProtocol.cmd << endl;
    cout << "----------------------------------------------------------" << endl;
}


/*
---------------------------------------------------------------------------------------------------------------------------------
*/

    // //////////////////// //
    // Internal Functions.  //
    // //////////////////// //

// Function to publish to a specific sender with a supplied Protocol
void sendToSender(BYTE sender, const ibo1_irc_api::Protocol& sendProtocol){

    // Checking which sender it should return to
    switch (sender)
    {
        case SENDER_SYSTEMSTATEMACHINE:{
            systemStateMachine_pub_ptr->publish(sendProtocol);
            break;
        }
        
        default:
            break;
    }
}


// Main Logic for wrapper
void wrapperLogic(vector<ChessEngineDefinitionStruct>& chessEngines){
    vector<BYTE> response;
    ibo1_irc_api::Protocol sendProtocol;

    BYTE gotCMD = returnedProtocol.cmd;

    switch(gotCMD){
        //Get chess engine names
        case CMD_INTERNAL_GETCHESSENGINES:{
            string chessEngineNames = "";
            for(ChessEngineDefinitionStruct ce : chessEngines){
                chessEngineNames += ce.name;
                chessEngineNames += "\u241f";
            }
            copy(chessEngineNames.begin(), chessEngineNames.end(), std::back_inserter(response));
            sendProtocol.cmd = CMD_INTERNAL_GETCHESSENGINES;
            sendProtocol.sender = SENDER_CHESSWRAPPER;
            sendProtocol.data = response;
            sendToSender(returnedProtocol.sender, sendProtocol);

            break;
        }

        case CMD_INTERNAL_STARTCHESSENGINE:{
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
                sendProtocol.cmd = ERROR_INTERNAL_CMD_CHESSENGINEDOESNTEXIST;
                sendProtocol.data = response;
                cout << "Couldnt find Chess Engine Name!" << endl;
            }else{
                cout << chessEnginePointer << endl;

                if(!(chessEnginePointer == 0)){
                    delete chessEnginePointer;
                }

                chessEnginePointer = new ChessEngine(toStartChessEngineFilePathName);
                sendProtocol.cmd = CMD_INTERNAL_STARTCHESSENGINE;
                cout << "Started Chess Engine!" << endl;
            }

            sendProtocol.sender = SENDER_CHESSWRAPPER;
            sendProtocol.data = response;
            sendToSender(returnedProtocol.sender, sendProtocol);
            break;
        }

        case CMD_INTERNAL_STOPCHESSENGINE:{
            if(chessEnginePointer == 0){
                sendProtocol.cmd == ERROR_INTERNAL_CMD_NOCHESSENGINERUNNING;
            }else{
                delete chessEnginePointer;
                chessEnginePointer = 0;
                sendProtocol.cmd == CMD_INTERNAL_STOPCHESSENGINE;
            }
            sendProtocol.sender = SENDER_CHESSWRAPPER;
            sendProtocol.data = response;
            sendToSender(returnedProtocol.sender, sendProtocol);
            break;
        }

        case CMD_INTERNAL_PLAYERMOVE:{
            if(chessEnginePointer == 0){
                sendProtocol.cmd == ERROR_INTERNAL_CMD_NOCHESSENGINERUNNING;
            }else{
                string moveCommand = "";
                DataCreator::convertBytesToString(returnedProtocol.data, moveCommand);

                cout << "Player Move send: " << moveCommand << endl;


                BYTE toReturnProtocolCmd;
                chessEnginePointer->playerMove(toReturnProtocolCmd, moveCommand);

                cout << "To return CmdByte: " << (int)toReturnProtocolCmd << endl;


                sendProtocol.cmd = toReturnProtocolCmd;
                sendProtocol.sender = SENDER_CHESSWRAPPER;
                sendProtocol.data = response;
                sendToSender(returnedProtocol.sender, sendProtocol);
                break;
            }
        }

        case CMD_INTERNAL_CHESSENGINEMOVE:{
            if(chessEnginePointer == NULL){
                sendProtocol.cmd == ERROR_INTERNAL_CMD_NOCHESSENGINERUNNING;
            }else{
                string chessEngineMove = "";
                BYTE toReturnProtocolCmd;

                chessEnginePointer->chessEngineMove(toReturnProtocolCmd, chessEngineMove);

                copy(chessEngineMove.begin(), chessEngineMove.end(), std::back_inserter(response));

                cout << "To return CmdByte: " << (int)toReturnProtocolCmd << endl;
                cout << "ChessEngine movement: " << chessEngineMove << endl;
                cout << "Current Board state as FEN: " << chessEnginePointer->getChessBoardFENString() << endl;
                cout << chessEnginePointer->getChessBoardString() << endl;

                sendProtocol.cmd = toReturnProtocolCmd;
                sendProtocol.sender = SENDER_CHESSWRAPPER;
                sendProtocol.data = response;
                sendToSender(returnedProtocol.sender, sendProtocol);
                break;
            }
        }

        case CMD_INTERNAL_LASTMOVECASTLEMOVE:{
            if(chessEnginePointer == NULL){
                sendProtocol.cmd == ERROR_INTERNAL_CMD_NOCHESSENGINERUNNING;
            }else{

                bool lastMoveCastleMove = chessEnginePointer->wasLastMoveCastleMove();
                BYTE toReturnProtocolCmd;

                if(lastMoveCastleMove){
                    toReturnProtocolCmd = CMD_INTERNAL_LASTMOVECASTLEMOVE;
                }
                else{
                    toReturnProtocolCmd = ERROR_INTERNAL_CMD_LASTMOVEWASNOTCASTLEMOVE;
                }

                sendProtocol.cmd = toReturnProtocolCmd;
                sendProtocol.sender = SENDER_CHESSWRAPPER;
                sendToSender(returnedProtocol.sender, sendProtocol);
                break;
            }
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

    ros::Subscriber server_sub = nh.subscribe("/ircChessWrapper", 1, &chessWrapperMessageReceived);

    ros::Publisher systemStateMachine_pub = nh.advertise<ibo1_irc_api::Protocol>("ircSystemStateMachine", 10);

    systemStateMachine_pub_ptr = &systemStateMachine_pub;

    vector<ChessEngineDefinitionStruct> chessEngines = FileHandler::readChessEngines(chessEnginesFilePathName);

    ChessBoard ch = ChessBoard();

    ros::Rate rate(10);

    //Waiting till ros system subscribers are activated
    // while(chessWrapper_pub.getNumSubscribers() == 0){
    //     rate.sleep();
    // }

    while(ros::ok()){

        wrapperLogic(chessEngines);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}