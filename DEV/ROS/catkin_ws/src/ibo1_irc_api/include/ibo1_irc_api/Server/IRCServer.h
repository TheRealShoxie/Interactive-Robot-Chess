#ifndef IRCSERVER_H
#define IRCSERVER_H

/*
 * IRCServer - Class which represents a IRCServer
 * <p>
 * This class enables the communication with the outside world.
 * It uses TCP socketing and implements the external Protocol described in the folder
 * data/Protocol/Protocol. 
 * 
 * <p>
 * 3rd party code is used in this class. The socketing was written with the help of:
 * https://gist.github.com/codehoose/020c6213f481aee76ea9b096acaddfaf
 * 
 * This includes following methods with slight adaption:
 *  - initiateServerSocket()
 *  - closeClientSocket()
 *  - recvBuffer()
 *  - sendBuffer()
 * 
 * Other 3rd party code used to convert bytes to integers was written with the help of:
 * https://stackoverflow.com/questions/34943835/convert-four-bytes-to-integer-using-c
 * 
 * This includes following methods:
 *  - convertBufferDataSizeToInt
 *  - convertIntToBufferDataSize
 * 
 * <p>
 * 
 * 
 * @author Omar Ibrahim
 * @version 0.1 ( Initial development ).
 * @version 1.0 ( Initial release ).
 * 
 * @see IRCServer.cpp
 * @see IRCServerNode.cpp
 * @see IRCServerNodeHelper.h
 * @see Protocol.h
*/


    // ////////// //
    // Includes.  //
    // ////////// //

// General includes
#include <iostream>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <vector>

// Protocol include
#include <ibo1_irc_api/ProtocolAPI/ProtocolDefinition.h>

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

        // Constructor for initiating IRCServer with a port
        IRCServer(unsigned int setPort);

        // //////// //
        // Methods. //
        // //////// //

        // Implementation of initiateServerSocket
        void initiateServerSocket();

        // Sets clientConnected bool to true
        void connectClient();

        // Method to close the client socket
        void closeClientSocket();

        // Method to extract the command from the buffer and returns bytes to read
        /*
            return meaning:
                * 
                * returns -1 when client not yet connected!
                * returns -2 if told to disconnect
                * returns -3 if cmd doesnt exist
                * returns -4 if bytes rescv arent length of proctocol(5)
        */
        int commandExtraction();

        // Used to send an answer to the client.
        int sendAnswer(vector<BYTE> replyData);

        // Gets the data from the input buffer as a vector and returns that.
        vector<BYTE> getData(int bufferSize);

        // ////////////////////// //
        // Read/Write properties. //
        // ////////////////////// //

        // Used to set the ClientCommand byte
        void setClientCommand(BYTE setClientCommand);

        // Method to get the ClientCommand byte
        BYTE getClientCommand();
        // ///////////////////// //
        // Read-only properties. //
        // ///////////////////// //

        // Method to get the port number
        unsigned int getPort();

        // Method to get the clientSocket number
        int getClientSocket();

        // Method to check if client is connected
        bool getClientConnected();

    private:

        // ////////////// //
        // Class methods. //
        // ////////////// //

        // Method to convert bufferDataSize to an int
        int convertBufferDataSizeToInt(vector<BYTE> buffer);

        // Method to convert int to bytes to use for DataSize 
        vector<BYTE> convertIntToBufferDataSize(int dataSize);

        // Method to send the buffer
        int sendBuffer(vector<BYTE> sendingBuffer);

        // Method to receive the buffer
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