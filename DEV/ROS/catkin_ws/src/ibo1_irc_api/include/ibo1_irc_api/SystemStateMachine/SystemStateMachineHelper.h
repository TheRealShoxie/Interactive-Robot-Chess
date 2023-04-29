#ifndef SYSTEMSTATEMACHINE_H
#define SYSTEMSTATEMACHINE_H

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
        static ibo1_irc_api::Protocol systemStateMachineChessEngineForwarder(ibo1_irc_api::Protocol& returnedProtocol, ros::Publisher* publisher){

            ibo1_irc_api::Protocol sendProtocolToChessWrapper;
            sendProtocolToChessWrapper.data = returnedProtocol.data;
            sendProtocolToChessWrapper.cmd = returnedProtocol.cmd;


            ibo1_irc_api::Protocol returnedProtocolFromNode = systemStateMachineForwarder(returnedProtocol, sendProtocolToChessWrapper, SENDER_CHESSWRAPPER, publisher);

            returnedProtocolFromNode.sender = SENDER_SYSTEMSTATEMACHINE;

            return returnedProtocolFromNode;
        }

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