#ifndef IRCSERVERNODEHELPER_H
#define IRCSERVERNODEHELPER_H

/*
 * IRCServerNodeHelper
 * <p>
 * This class is used as a helper class for the ServerNode.cpp
 * Used to define functions to not cluster the ServerNode.cpp file.
 * It provides logging in for users and converting internal to external protocols.
 * 
 * The definitions for these can be seen in the folder data/Protocol/...
 * 
 * <p>
 * 
 * 
 * @author Omar Ibrahim
 * @version 0.1 ( Initial development ).
 * @version 1.0 ( Initial release ).
 * 
 * @see IRCServerNode.cpp
 * @see internalProtocol.h
 * @see Protocol.h
 * @see Users.h
 * @see IRCServer.h
*/


    // ////////// //
    // Includes.  //
    // ////////// //

#include <ros/ros.h>

#include <ibo1_irc_api/Protocol.h>


#include <ibo1_irc_api/User/Users.h>
#include <ibo1_irc_api/Server/IRCServer.h>
#include <ibo1_irc_api/ProtocolAPI/InternalProtocolDefinition.h>
   
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

        // Method to login a user
        static void cmdUserLogin(const vector<BYTE>& receivedData, IRCServer &server, Users &users, vector<BYTE>& answer){

            // Tries to login a users and sets the answer to true or false if the user is an admin
            try{
                User user = User(receivedData);
                User foundUser = users.findUser(user);

                if(foundUser.isAdmin()) answer.push_back(0x01);
                else answer.push_back(0x00);
            }
            // If it fails most likely the user doesn't exist so set cmd user does not exist
            catch(runtime_error er){
                server.setClientCommand(ERROR_CMD_USERDOESNTEXIST);
            }
        }

        // Converts internal cmd to external cmd
        static BYTE internalToExternalCmdConverter(const BYTE& internalCmdToConvert){
            switch (internalCmdToConvert)
            {
                case CMD_INTERNAL_SETTARGET:
                    return ERROR_CMD_UNRECOGNIZABLE;
                    break;

                case CMD_INTERNAL_CLEARTARGET:
                    return ERROR_CMD_UNRECOGNIZABLE;
                    break;

                case CMD_INTERNAL_ROBOTARMMOVE:
                    return ERROR_CMD_UNRECOGNIZABLE;
                    break;

                case CMD_INTERNAL_GETCHESSENGINES:
                    return CMD_GETCHESSENGINES;
                    break;

                case CMD_INTERNAL_STARTCHESSENGINE:
                    return CMD_STARTCHESSENGINE;
                    break;

                case CMD_INTERNAL_STOPCHESSENGINE:
                    return CMD_STOPCHESSENGINE;
                    break;

                case CMD_INTERNAL_GETCHESSENGINEOPTIONS:
                    return CMD_GETCHESSENGINEOPTIONS;
                    break;

                case CMD_INTERNAL_SETCHESSENGINEOPTIONS:
                    return CMD_SETCHESSENGINEOPTIONS;
                    break;

                case CMD_INTERNAL_SETSEARCHOPTIONS:
                    return CMD_SETSEARCHOPTIONS;
                    break;

                case CMD_INTERNAL_PLAYERMOVE:
                    return CMD_PLAYERMOVE;
                    break;

                case CMD_INTERNAL_CHESSENGINEMOVE:
                    return CMD_CHESSENGINEMOVE;
                    break;

                case CMD_INTERNAL_LASTMOVECASTLEMOVE:
                    return ERROR_CMD_UNRECOGNIZABLE;
                    break;

                case CMD_INTERNAL_SYSTEMWITHOUTSIM:
                    return CMD_SYSTEMWITHOUTSIM;
                    break;

                case CMD_INTERNAL_SYSTEMFULLSIM:
                    return CMD_SYSTEMFULLSIM;
                    break;
                
                default:
                    return ERROR_CMD_UNRECOGNIZABLE;
                    break;
            }
        }

        // Converts external cmd to internal cmd
        static BYTE externalToInternalCmdConverter(const BYTE& externalCmdToConver){
            switch (externalCmdToConver)
            {
                
                case CMD_DISCONNECT:
                    return ERROR_INTERNAL_CMD_UNRECOGNIZABLE;
                    break;

                case CMD_CONNECT:
                    return ERROR_INTERNAL_CMD_UNRECOGNIZABLE;
                    break;

                case CMD_LOGIN:
                    return ERROR_INTERNAL_CMD_UNRECOGNIZABLE;
                    break;

                case CMD_CREATEUSER:
                    return ERROR_INTERNAL_CMD_UNRECOGNIZABLE;
                    break;

                case CMD_GETCHESSENGINES:
                    return CMD_INTERNAL_GETCHESSENGINES;
                    break;

                case CMD_STARTCHESSENGINE:
                    return CMD_INTERNAL_STARTCHESSENGINE;
                    break;

                case CMD_STOPCHESSENGINE:
                    return CMD_INTERNAL_STOPCHESSENGINE;
                    break;

                case CMD_GETCHESSENGINEOPTIONS:
                    return CMD_INTERNAL_GETCHESSENGINEOPTIONS;
                    break;

                case CMD_SETCHESSENGINEOPTIONS:
                    return CMD_INTERNAL_SETCHESSENGINEOPTIONS;
                    break;

                case CMD_SETSEARCHOPTIONS:
                    return CMD_INTERNAL_SETSEARCHOPTIONS;
                    break;

                case CMD_PLAYERMOVE:
                    return CMD_INTERNAL_PLAYERMOVE;
                    break;

                case CMD_CHESSENGINEMOVE:
                    return CMD_INTERNAL_CHESSENGINEMOVE;
                    break;

                case CMD_SYSTEMWITHOUTSIM:
                    return CMD_INTERNAL_SYSTEMWITHOUTSIM;
                    break;

                case CMD_SYSTEMFULLSIM:
                    return CMD_INTERNAL_SYSTEMFULLSIM;
                    break;
                
                default:
                    return ERROR_INTERNAL_CMD_UNRECOGNIZABLE;
                    break;
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
        
        // //////////////// //
        // Class variables. //
        // //////////////// //

        // /////////////////// //
        // Instance variables. //
        // /////////////////// //

    
};
#endif //IRCSERVERNODEHELPER_H