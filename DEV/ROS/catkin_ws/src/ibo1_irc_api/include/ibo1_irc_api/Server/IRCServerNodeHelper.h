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