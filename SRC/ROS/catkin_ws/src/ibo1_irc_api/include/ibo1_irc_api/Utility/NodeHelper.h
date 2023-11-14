#ifndef NODEHELPER_H
#define NODEHELPER_H

/*
 * NodeHelper
 * <p>
 * The NodeHelper class is used for general node helping.
 * 
 * Its current main implementation is used for forwarding protocols between nodes and defining the expected returns from those nodes where the communication
 * started.
 * 
 * 
 * @author Omar Ibrahim
 * @version 0.1 ( Initial development ).
 * @version 1.0 ( Initial release ).
 * 
 * @see SystemStateMachineHelper.h
 * @see ServerNode.cpp
 * @see SystemStateMachineNode.cpp
*/

    // ////////// //
    // Includes.  //
    // ////////// //

#include <ros/ros.h>

#include <ibo1_irc_api/Protocol.h>

#include <ibo1_irc_api/ProtocolAPI/InternalProtocolDefinition.h>
   
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

        // Method for forwarding internal protocols inside the system
        static void forwarder(ibo1_irc_api::Protocol& returnedProtocol, const vector<BYTE>& dataToSend, const BYTE& protocolCmd, const BYTE& sender, const BYTE& receiveFrom, vector<BYTE>& returnedInternalProtocolData, 
                              BYTE& returnedCMD, ros::Publisher* publisher){

            //Calling internal forwarder which sends a protocol to the supplied publisher
            internalForwarder(dataToSend, protocolCmd, sender, publisher);


            //Getting the expected return from internal command send
            vector<BYTE> expectedReturn = getExpectedReturnInternal(protocolCmd);


            bool foundExpectedCmd = false;

            //While we did not find the expected return keep going
            // Currently there is a possibility we will go into a infinite loop
            // This system needs to be revamped
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

        // Method that forwards a protocol from supplied data, cmd and the sender to the supplied publisher
        static void internalForwarder(const vector<BYTE>& sendData, const BYTE& cmd, const BYTE &sender, ros::Publisher* publisher){
            ibo1_irc_api::Protocol forwardProtocol;
            forwardProtocol.cmd = cmd;
            forwardProtocol.sender = sender;
            forwardProtocol.data = sendData;
            

            publisher->publish(forwardProtocol);
        }


        // This method checks if the supplied received Protocol cmd matches the supplied expected values. It then overrides the protocol data and cmd if it finds a match
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
                    
                    expectedReturn.push_back(ERROR_INTERNAL_CMD_SYSTEMINPLAYCHESSSTATEMACHINE);
                    expectedReturn.push_back(ERROR_INTERNAL_CMD_UNRECOGNIZABLE);

                    return expectedReturn;
                }

            case CMD_INTERNAL_STARTCHESSENGINE:{

                    expectedReturn.push_back(CMD_INTERNAL_STARTCHESSENGINE);
                    expectedReturn.push_back(ERROR_INTERNAL_CMD_CHESSENGINEDOESNTEXIST);

                    expectedReturn.push_back(ERROR_INTERNAL_CMD_SYSTEMINPLAYCHESSSTATEMACHINE);
                    expectedReturn.push_back(ERROR_INTERNAL_CMD_UNRECOGNIZABLE);

                    return expectedReturn;
                }

            case CMD_INTERNAL_STOPCHESSENGINE:{

                    expectedReturn.push_back(CMD_INTERNAL_STOPCHESSENGINE);
                    expectedReturn.push_back(ERROR_INTERNAL_CMD_NOCHESSENGINERUNNING);

                    expectedReturn.push_back(ERROR_INTERNAL_CMD_SYSTEMINPLAYCHESSSTATEMACHINE);
                    expectedReturn.push_back(ERROR_INTERNAL_CMD_UNRECOGNIZABLE);

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
                    expectedReturn.push_back(ERROR_INTERNAL_CMD_NOCHESSBOARDINFORMATION);
                    expectedReturn.push_back(ERROR_INTERNAL_CMD_PICKUPCELLEMPTY);

                    expectedReturn.push_back(ERROR_INTERNAL_CMD_SYSTEMINPLAYCHESSSTATEMACHINE);
                    expectedReturn.push_back(ERROR_INTERNAL_CMD_UNRECOGNIZABLE);

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
                    expectedReturn.push_back(ERROR_INTERNAL_CMD_NOCHESSBOARDINFORMATION);
                    expectedReturn.push_back(ERROR_INTERNAL_CMD_PICKUPCELLEMPTY);

                    expectedReturn.push_back(ERROR_INTERNAL_CMD_SYSTEMINPLAYCHESSSTATEMACHINE);
                    expectedReturn.push_back(ERROR_INTERNAL_CMD_UNRECOGNIZABLE);

                    return expectedReturn;
                }

            case CMD_INTERNAL_SYSTEMFULLSIM:{
                expectedReturn.push_back(CMD_INTERNAL_SYSTEMFULLSIM);
                expectedReturn.push_back(ERROR_INTERNAL_CMD_SYSTEMINPLAYCHESSSTATEMACHINE);
                expectedReturn.push_back(ERROR_INTERNAL_CMD_UNRECOGNIZABLE);

                return expectedReturn;
            }

            case CMD_INTERNAL_SYSTEMWITHOUTSIM:{
                expectedReturn.push_back(CMD_INTERNAL_SYSTEMWITHOUTSIM);
                expectedReturn.push_back(ERROR_INTERNAL_CMD_SYSTEMINPLAYCHESSSTATEMACHINE);
                expectedReturn.push_back(ERROR_INTERNAL_CMD_UNRECOGNIZABLE);
                
                return expectedReturn;
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