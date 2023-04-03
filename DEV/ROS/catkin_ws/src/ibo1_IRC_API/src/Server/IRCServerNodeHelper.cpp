#include "ibo1_IRC_API/Server/IRCServerNodeHelper.h"

    // ///////////// //
    // Constructors. //
    // ///////////// //

    // ////////////// //
    // Class methods. //
    // ////////////// //

    // ////////////////////// //
    // Read/Write properties. //
    // ////////////////////// //

    // ///////////////////// //
    // Read-only properties. //
    // ///////////////////// //

    // //////// //
    // Methods. //
    // //////// //

    void IRCServerNodeHelper::cmdUserLogin(const vector<BYTE>& receivedData, IRCServer &server, Users &users, vector<BYTE>& answer){
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

    void IRCServerNodeHelper::forwarderChessWrapper(const vector<BYTE>& receivedData, BYTE cmd, ros::Publisher &server_pub){
        ibo1_IRC_API::Protocol forwardProtocol;
        forwardProtocol.cmd = cmd;

        forwardProtocol.data = receivedData;

        server_pub.publish(forwardProtocol);
    }

    bool IRCServerNodeHelper::receiverChessWrapper(const ibo1_IRC_API::Protocol receivedFromChessWrapper, BYTE &returnedCMD, vector<BYTE>& returnedInternalProtocol, const vector<BYTE>& expectedReturn){

        returnedCMD  = receivedFromChessWrapper.cmd;

        for(auto &byte : expectedReturn){
            if(byte == returnedCMD){
                returnedInternalProtocol = receivedFromChessWrapper.data;
                return true;
            }
        }

        return false;
    }