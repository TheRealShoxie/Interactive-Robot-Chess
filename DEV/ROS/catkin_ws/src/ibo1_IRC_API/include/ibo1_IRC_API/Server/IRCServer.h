#ifndef IRCSERVER_H
#define IRCSERVER_H

#include <iostream>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <vector>

using namespace std;


// Used for throwing exceptions
#include <stdexcept>


    // ////////// //
    // Constants. //
    // ////////// //

// Datatype
typedef std::uint8_t BYTE;

// Protocol Commands
#define CMD_DISCONNECT              (BYTE)0x00
#define CMD_CONNECT                 (BYTE)0x01
#define CMD_LOGIN                   (BYTE)0x02
#define CMD_CREATEUSER              (BYTE)0x03






// Error Codes:
#define ERROR_CMD_UNRECOGNIZABLE    (BYTE)0xff
#define ERROR_CONNECT               (BYTE)0xfe
#define ERROR_CMD_USERDOESNTEXIST   (BYTE)0xfd
#define EROR_CMD_USERALREADYEXISTS  (BYTE)0xfc
#define ERROR_CMD_USERNOTCREATED    (BYTE)0xfb


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
        int sendAnswer(std::vector<BYTE> replyData);
        std::vector<BYTE> getData(int bufferSize);

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

        int convertBufferDataSizeToInt(std::vector<BYTE> buffer);
        std::vector<BYTE> convertIntToBufferDataSize(int dataSize);
        int sendBuffer(std::vector<BYTE> sendingBuffer);
        int receiveBuffer(std::vector<BYTE>& receivingBuffer);
        
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