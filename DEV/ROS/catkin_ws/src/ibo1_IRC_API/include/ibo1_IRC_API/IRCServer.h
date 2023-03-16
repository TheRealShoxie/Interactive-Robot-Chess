#ifndef IRCSERVER_H
#define IRCSERVER_H

#include <iostream>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <string.h>
#include <string>
#include <vector>


// Used for throwing exceptions
#include <stdexcept>

// Datatype
typedef std::uint8_t BYTE;

// Protocol Commands
#define CMD_CONNECT             (BYTE)0x00
#define CMD_STRING              (BYTE)0x01
#define CMD_LOGIN               (BYTE)0x02
#define CMD_DISCONNECT          (BYTE)0xff

// Error Codes:
#define ERROR_CONNECT           (BYTE)0xfe



class IRCServer{
    public:
        IRCServer(unsigned int setPort);
        void initiateServerSocket();
        void connectClient();
        void closeClientSocket();
        int commandExtraction();
        int commandAnswer(std::vector<BYTE> replyData);
        std::vector<BYTE> getData(int bufferSize);

        unsigned int getPort();
        int getClientSocket();
        BYTE getClientCommand();

    private:
        int convertBufferDataSizeToInt(std::vector<BYTE> buffer);
        std::vector<BYTE> convertIntToBufferDataSize(int dataSize);
        int sendBuffer(std::vector<BYTE> sendingBuffer);
        int receiveBuffer(std::vector<BYTE>& receivingBuffer);
        
        int clientSocket;
        BYTE clientCommand;
        unsigned int port;
        bool clientConnected = false;



};
#endif //IRCSERVER_H