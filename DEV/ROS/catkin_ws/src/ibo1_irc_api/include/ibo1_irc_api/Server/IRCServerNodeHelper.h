#ifndef IRCSERVERNODEHELPER_H
#define IRCSERVERNODEHELPER_H

#include <ros/ros.h>

#include <ibo1_irc_api/Protocol.h>


#include <ibo1_irc_api/User/Users.h>
#include <ibo1_irc_api/Server/IRCServer.h>
   
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
        static void cmdUserLogin(const vector<BYTE>& receivedData, IRCServer &server, Users &users, vector<BYTE>& answer){
            cout << "-------UserLogin-------" << endl;

            try{
                User user = User(receivedData);
                User foundUser = users.findUser(user);

                
                cout << "User requested: " << user.toString() << endl;
                cout << "User found: \n" << foundUser.toString() << endl;
                cout << "User admin rights:" << foundUser.isAdmin() << endl;

                if(foundUser.isAdmin()) answer.push_back(0x01);
                else answer.push_back(0x00);
            }catch(runtime_error er){
                cout << "User was not found!" << endl;
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