#ifndef IRCSERVERNODEHELPER_H
#define IRCSERVERNODEHELPER_H

#include <ros/ros.h>

#include <ibo1_IRC_API/Protocol.h>


#include <ibo1_IRC_API/User/Users.h>
#include <ibo1_IRC_API/Server/IRCServer.h>
   
    // ////////// //
    // Constants. //
    // ////////// //

class IRCServerNodeHelper{

    public:

        // ///////////// //
        // Constructors. //
        // ///////////// //

        // //////// //
        // Methods. //
        // //////// //
        static void cmdUserLogin(const vector<BYTE>& receivedData, IRCServer *server, Users *users, vector<BYTE>& answer);
        static void forwarderChessWrapper(const vector<BYTE>& receivedData, const BYTE cmd, ros::Publisher *server_pub);
        static bool receiverChessWrapper(const ibo1_IRC_API::Protocol receivedFromChessEngineWrapper, BYTE &returnedCMD, vector<BYTE>& returnedInternalProtocol, const vector<BYTE>& expectedReturn);

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
#endif //IRCSERVERNODEHELPER_H