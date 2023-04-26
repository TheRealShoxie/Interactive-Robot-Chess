#ifndef COMMANDEXECUTERHELPER_H
#define COMMANDEXECUTERHELPER_H

#include <ros/ros.h>

#include <ibo1_irc_api/Protocol.h>

#include <ibo1_irc_api/ProtocolAPI/InternalProtocolDefinition.h>
#include <ibo1_irc_api/Utility/NodeHelper.h>
#include <vector>
   
    // ////////// //
    // Constants. //
    // ////////// //

class CommandExecuterHelper{

    public:

        // ///////////// //
        // Constructors. //
        // ///////////// //

        // //////// //
        // Methods. //
        // //////// //

        // Method logic for getting chessEngine names and sending it back to who requested it
        static ibo1_irc_api::Protocol chessEngineForwarder(ibo1_irc_api::Protocol& returnedProtocol, ros::Publisher* publisher){
            ibo1_irc_api::Protocol sendProtocol;

            NodeHelper::forwarder(returnedProtocol, returnedProtocol.data, returnedProtocol.cmd, SENDER_COMMANDEXECUTER, SENDER_CHESSWRAPPER, sendProtocol.data, sendProtocol.cmd, publisher);


            sendProtocol.sender = SENDER_COMMANDEXECUTER;
            return sendProtocol;
        }

        static ibo1_irc_api::Protocol commandExecuterForwarder(ibo1_irc_api::Protocol& returnedProtocol, const BYTE& receiveFrom, ros::Publisher* publisher){
            ibo1_irc_api::Protocol sendProtocol;

            NodeHelper::forwarder(returnedProtocol, returnedProtocol.data, returnedProtocol.cmd, SENDER_COMMANDEXECUTER, receiveFrom, sendProtocol.data, sendProtocol.cmd, publisher);


            sendProtocol.sender = SENDER_COMMANDEXECUTER;
            return sendProtocol;
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

#endif //COMMANDEXECUTERHELPER_H