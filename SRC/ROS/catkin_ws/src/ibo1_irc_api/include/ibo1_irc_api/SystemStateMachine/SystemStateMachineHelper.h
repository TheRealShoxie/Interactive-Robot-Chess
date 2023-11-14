#ifndef SYSTEMSTATEMACHINE_H
#define SYSTEMSTATEMACHINE_H

/*
 * SystemStateMachineHelper
 * <p>
 * This class is used as a helper class for the SystemStateMachineNode.cpp
 * Used to define functions to not cluster the SystemStateMachineNode.cpp file.
 * It provides forwarding of messages using the NodeHelper.h but specified generally and
 * specifically for the ChessWrapper. This enables the use of the NodeHelper function with less inputs as they
 * are already defined
 * 
 * <p>
 * 
 * 
 * @author Omar Ibrahim
 * @version 0.1 ( Initial development ).
 * @version 1.0 ( Initial release ).
 * 
 * @see SystemStateMachineNode.cpp
 * @see InternalProtocolDefinition.h
 * @see NodeHelper.h
*/


    // ////////// //
    // Includes.  //
    // ////////// //

#include <ros/ros.h>

#include <ibo1_irc_api/Protocol.h>

#include <ibo1_irc_api/ProtocolAPI/InternalProtocolDefinition.h>
#include <ibo1_irc_api/Utility/NodeHelper.h>
#include <vector>
   
    // ////////// //
    // Constants. //
    // ////////// //

class SystemStateMachineHelper{

    public:

        // ///////////// //
        // Constructors. //
        // ///////////// //

        // //////// //
        // Methods. //
        // //////// //

        // Method logic for getting chessEngine names and sending it back to who requested it
        static ibo1_irc_api::Protocol systemStateMachineChessWrapperForwarder(ibo1_irc_api::Protocol& returnedProtocol, ros::Publisher* publisher){

            ibo1_irc_api::Protocol sendProtocolToChessWrapper;
            sendProtocolToChessWrapper.data = returnedProtocol.data;
            sendProtocolToChessWrapper.cmd = returnedProtocol.cmd;

            // Using general statemachine forwarder
            ibo1_irc_api::Protocol returnedProtocolFromNode = systemStateMachineForwarder(returnedProtocol, sendProtocolToChessWrapper, SENDER_CHESSWRAPPER, publisher);

            returnedProtocolFromNode.sender = SENDER_SYSTEMSTATEMACHINE;

            return returnedProtocolFromNode;
        }


        // Method for forwarding messages from the SystemStateMachineNode
        static ibo1_irc_api::Protocol systemStateMachineForwarder(ibo1_irc_api::Protocol& returnedProtocol, ibo1_irc_api::Protocol& protocolToSend, const BYTE& recvFrom, ros::Publisher* publisher){
            ibo1_irc_api::Protocol returnedProtocolFromNode;

            NodeHelper::forwarder(returnedProtocol, protocolToSend.data, protocolToSend.cmd, SENDER_SYSTEMSTATEMACHINE,
                                 recvFrom, returnedProtocolFromNode.data, returnedProtocolFromNode.cmd, publisher);

            returnedProtocolFromNode.sender = recvFrom;

            return returnedProtocolFromNode;
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
        
        // //////////////// //
        // Class variables. //
        // //////////////// //

        // /////////////////// //
        // Instance variables. //
        // /////////////////// //

    
};

#endif //SYSTEMSTATEMACHINE_H