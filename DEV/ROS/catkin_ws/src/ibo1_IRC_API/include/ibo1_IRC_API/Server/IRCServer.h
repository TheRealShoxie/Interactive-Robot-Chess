#ifndef IRCSERVER_H
#define IRCSERVER_H

#include <iostream>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <vector>

#include <ibo1_IRC_API/ProtocolAPI/ProtocolDefinition.h>

using namespace std;


// Used for throwing exceptions
#include <stdexcept>


    // ////////// //
    // Constants. //
    // ////////// //


class IRCServer{
    public:
        
        // ///////////// //
        // Constructors. //
        // ///////////// //

        IRCServer(unsigned int setPort);

        // //////// //
        // Methods. //
        // //////// //

        void initiateServerSocket();
        void connectClient();
        void closeClientSocket();
        int commandExtraction();
        int sendAnswer(vector<BYTE> replyData);
        vector<BYTE> getData(int bufferSize);

        // ////////////////////// //
        // Read/Write properties. //
        // ////////////////////// //
        void setClientCommand(BYTE setClientCommand);
        BYTE getClientCommand();
        // ///////////////////// //
        // Read-only properties. //
        // ///////////////////// //
        unsigned int getPort();
        int getClientSocket();
        bool getClientConnected();

    private:

        // ////////////// //
        // Class methods. //
        // ////////////// //

        int convertBufferDataSizeToInt(vector<BYTE> buffer);
        vector<BYTE> convertIntToBufferDataSize(int dataSize);
        int sendBuffer(vector<BYTE> sendingBuffer);
        int receiveBuffer(vector<BYTE>& receivingBuffer);
        
        // //////////////// //
        // Class variables. //
        // //////////////// //

        // /////////////////// //
        // Instance variables. //
        // /////////////////// //

        int clientSocket;
        BYTE clientCommand;
        unsigned int port;
        bool clientConnected = false;



};
#endif //IRCSERVER_H