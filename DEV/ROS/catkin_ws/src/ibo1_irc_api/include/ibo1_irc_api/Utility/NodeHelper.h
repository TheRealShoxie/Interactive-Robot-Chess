#ifndef NODEHELPER_H
#define NODEHELPER_H

#include <ros/ros.h>

#include <ibo1_irc_api/Protocol.h>
   
    // ////////// //
    // Constants. //
    // ////////// //

class NodeHelper{

    public:

        // ///////////// //
        // Constructors. //
        // ///////////// //

        // //////// //
        // Methods. //
        // //////// //


        static void forwarder(ibo1_irc_api::Protocol& returnedProtocol, const vector<BYTE>& dataToSend, const BYTE& protocolCmd, const BYTE& sender, const BYTE& receiveFrom, vector<BYTE>& returnedInternalProtocolData, 
                              BYTE& returnedCMD, ros::Publisher* publisher){
            internalForwarder(dataToSend, protocolCmd, sender, publisher);

            vector<BYTE> expectedReturn = getExpectedReturnInternal(protocolCmd);


            bool foundExpectedCmd = false;

            while(!foundExpectedCmd){

                if(returnedProtocol.sender == receiveFrom){
                    foundExpectedCmd = internalReceiver(returnedProtocol, returnedCMD, returnedInternalProtocolData, expectedReturn);
                }
                ros::spinOnce();
            }
        }

        

        // ////////////////////// //
        // Read/Write properties. //
        // ////////////////////// //

        // ///////////////////// //
        // Read-only properties. //
        // ///////////////////// //

    private:

        // ////////////// //
        // Class methods. //
        // ////////////// //
        static void internalForwarder(const vector<BYTE>& sendData, const BYTE& cmd, const BYTE &sender, ros::Publisher* publisher){
            ibo1_irc_api::Protocol forwardProtocol;
            forwardProtocol.cmd = cmd;
            forwardProtocol.sender = sender;
            forwardProtocol.data = sendData;
            

            publisher->publish(forwardProtocol);
        }

        static bool internalReceiver(const ibo1_irc_api::Protocol& received, BYTE &returnedCMD, vector<BYTE>& returnedInternalProtocolData, const vector<BYTE>& expectedReturn){
            returnedCMD  = received.cmd;

            for(auto &byte : expectedReturn){
                if(byte == returnedCMD){
                    returnedInternalProtocolData = received.data;
                    return true;
                }
            }

            return false;
        }

        //Returns expectedReturns for internal use only
        static vector<BYTE> getExpectedReturnInternal(const BYTE &protocolCMD){

            //Declaring expectedReturn
            vector<BYTE> expectedReturn;
            
            switch (protocolCMD)
            {
                case CMD_INTERNAL_GETCHESSENGINES:{
                    expectedReturn.push_back(CMD_INTERNAL_GETCHESSENGINES);

                    return expectedReturn;
                }

            case CMD_INTERNAL_STARTCHESSENGINE:{

                    expectedReturn.push_back(CMD_INTERNAL_STARTCHESSENGINE);
                    expectedReturn.push_back(ERROR_INTERNAL_CMD_CHESSENGINEDOESNTEXIST);

                    return expectedReturn;
                }

            case CMD_INTERNAL_STOPCHESSENGINE:{

                    expectedReturn.push_back(CMD_INTERNAL_STOPCHESSENGINE);
                    expectedReturn.push_back(ERROR_INTERNAL_CMD_NOCHESSENGINERUNNING);

                    return expectedReturn;
                }

            case CMD_INTERNAL_PLAYERMOVE:{

                    expectedReturn.push_back(CMD_INTERNAL_PLAYERMOVE);
                    expectedReturn.push_back(ERROR_INTERNAL_CMD_NOCHESSENGINERUNNING);

                    expectedReturn.push_back(ERROR_INTERNAL_CMD_PAWNCOLLIDEDSTRAIGHT);
                    expectedReturn.push_back(ERROR_INTERNAL_CMD_PAWNCOLLIDEDDIAGONALOREMPTYCELL);
                    expectedReturn.push_back(ERROR_INTERNAL_CMD_STARTINGCELLEMPTY);
                    expectedReturn.push_back(ERROR_INTERNAL_CMD_NOTTHATCOLORSTURN);
                    expectedReturn.push_back(ERROR_INTERNAL_CMD_MOVEINVALIDORBLOCKEDBYSAMECOLOR);
                    expectedReturn.push_back(ERROR_INTERNAL_CMD_CANNOTCASTLEKINGSIDE);
                    expectedReturn.push_back(ERROR_INTERNAL_CMD_CANNOTCASTLEQUEENSIDE);
                    expectedReturn.push_back(ERROR_INTERNAL_CMD_OWNKINGINCHECK);
                    expectedReturn.push_back(ERROR_INTERNAL_CMD_OTHERKINGINCHECKMATE);
                    expectedReturn.push_back(ERROR_INTERNAL_CMD_PAWNNOTALLOWEDNOTPROMOTIONMOVE);
                    expectedReturn.push_back(ERROR_INTERNAL_CMD_PIECETOPROMOTEISNOTPAWN);
                    expectedReturn.push_back(ERROR_INTERNAL_CMD_PAWNNOTMOVINGTOENDOFBOARD);
                    expectedReturn.push_back(ERROR_INTERNAL_CMD_INVALIDPIECENAMETOPROMOTEINTO);
                    expectedReturn.push_back(ERROR_INTERNAL_CMD_INVALIDMOVEFORMAT);

                    return expectedReturn;
                }

            case CMD_INTERNAL_CHESSENGINEMOVE:{

                    expectedReturn.push_back(CMD_INTERNAL_CHESSENGINEMOVE);
                    expectedReturn.push_back(ERROR_INTERNAL_CMD_NOCHESSENGINERUNNING);

                    expectedReturn.push_back(ERROR_INTERNAL_CMD_PAWNCOLLIDEDSTRAIGHT);
                    expectedReturn.push_back(ERROR_INTERNAL_CMD_PAWNCOLLIDEDDIAGONALOREMPTYCELL);
                    expectedReturn.push_back(ERROR_INTERNAL_CMD_STARTINGCELLEMPTY);
                    expectedReturn.push_back(ERROR_INTERNAL_CMD_NOTTHATCOLORSTURN);
                    expectedReturn.push_back(ERROR_INTERNAL_CMD_CANNOTCASTLEQUEENSIDE);
                    expectedReturn.push_back(ERROR_INTERNAL_CMD_OWNKINGINCHECK);
                    expectedReturn.push_back(ERROR_INTERNAL_CMD_OTHERKINGINCHECKMATE);
                    expectedReturn.push_back(ERROR_INTERNAL_CMD_PAWNNOTALLOWEDNOTPROMOTIONMOVE);
                    expectedReturn.push_back(ERROR_INTERNAL_CMD_PIECETOPROMOTEISNOTPAWN);
                    expectedReturn.push_back(ERROR_INTERNAL_CMD_PAWNNOTMOVINGTOENDOFBOARD);
                    expectedReturn.push_back(ERROR_INTERNAL_CMD_INVALIDPIECENAMETOPROMOTEINTO);
                    expectedReturn.push_back(ERROR_INTERNAL_CMD_CHESSENGINECREATEDNOMOVE);
                    expectedReturn.push_back(ERROR_INTERNAL_CMD_INVALIDMOVEFORMAT);

                    return expectedReturn;
                }

            case CMD_INTERNAL_SETTARGET:{
                expectedReturn.push_back(CMD_INTERNAL_SETTARGET);
                expectedReturn.push_back(ERROR_INTERNAL_CMD_INVALIDMOVEFORMAT);
            }

            case CMD_INTERNAL_CLEARTARGET:{
                expectedReturn.push_back(CMD_INTERNAL_CLEARTARGET);
            }

            default:
                return expectedReturn;
            }
        }
        
        // //////////////// //
        // Class variables. //
        // //////////////// //

        // /////////////////// //
        // Instance variables. //
        // /////////////////// //

    
};
#endif //NODEHELPER_H