
#include "ibo1_IRC_API/IRCServer.h"


    // ///////////// //
    // Constructors. //
    // ///////////// //

IRCServer::IRCServer(unsigned int setPort) : clientSocket(-1){
    clientCommand = CMD_CONNECT;
    port = setPort;
}


    // ////////////// //
    // Class methods. //
    // ////////////// //


//Implementation of closing client Socket
void IRCServer::closeClientSocket(){
    close(clientSocket);
}

// Implementation of client connected
void IRCServer::connectClient(){
    clientConnected = true;
}

//Implementation of Sending method
int IRCServer::sendBuffer(std::vector<BYTE> sendingBuffer){
    return send(clientSocket, &sendingBuffer[0], sendingBuffer.size(), 0);
}

//Implementation of Receiving method
int IRCServer::receiveBuffer(std::vector<BYTE>& receivingBuffer){

    // Waiting for a message
    return recv(clientSocket, &receivingBuffer[0], receivingBuffer.size(), 0);
}


// https://stackoverflow.com/questions/34943835/convert-four-bytes-to-integer-using-c
int IRCServer::convertBufferDataSizeToInt(std::vector<BYTE> buffer){

    return int( (buffer[1]) << 24 |
                (buffer[2]) << 16 |
                (buffer[3]) << 8 |
                (buffer[4]));
}

std::vector<BYTE> IRCServer::convertIntToBufferDataSize(int dataSize){
    std::vector<BYTE> intAsBytes;

    intAsBytes.push_back(dataSize >> 24);
    intAsBytes.push_back(dataSize >> 16);
    intAsBytes.push_back(dataSize >> 8);
    intAsBytes.push_back(dataSize);

    return intAsBytes;
}


    // ////////////////////// //
    // Read/Write properties. //
    // ////////////////////// //


unsigned int IRCServer::getPort(){
    return port;
}

int IRCServer::getClientSocket(){
    return clientSocket;
}

BYTE IRCServer::getClientCommand(){
    return clientCommand;
}


    // //////// //
    // Methods. //
    // //////// //


// Implementation of initiateServerSocket
void IRCServer::initiateServerSocket(){
    // Creating server socket
    int listeningSocket = socket(AF_INET, SOCK_STREAM, 0);
    if (listeningSocket == -1) throw std::runtime_error("Couldn't initialize socket!");

    std::cout << "Starting...." << std::endl;

    // Binding the socket to a IP Address and port
    sockaddr_in hint;
    hint.sin_family = AF_INET;
    // htons = host to network host
    hint.sin_port = htons(port);
    // inet_pton converts a pointer string of number to an array of integers
    // We use "0.0.0.0" for any address
    inet_pton(AF_INET, "0.0.0.0", &hint.sin_addr);

    if(bind(listeningSocket, (sockaddr *)&hint, sizeof(hint)) == -1) throw std::runtime_error("Couldn't bin Socket to IP Address or Port!");


    // Marking the socket for listening in
    if(listen(listeningSocket, SOMAXCONN) == -1) throw std::runtime_error("Cannot listen!");


    // Accepting a client
    sockaddr_in client;
    socklen_t clientSize = sizeof(client);
    char host[NI_MAXHOST];
    char service[NI_MAXSERV];

    // Accepting an incoming client
    clientSocket = accept(listeningSocket, 
                          (sockaddr *)&client,
                          &clientSize);
            
    if (clientSocket == -1) throw std::runtime_error("Problem with client connecting!");
}


std::vector<BYTE> IRCServer::getData(int bufferSize){
    std::vector<BYTE> dataBuffer;
    dataBuffer.resize(bufferSize);
    int bytesRecv = receiveBuffer(dataBuffer);

    return dataBuffer;
}


int IRCServer::commandExtraction(){


    std::vector<BYTE> cmdBuffer;
    cmdBuffer.resize(5);
    
    std::cout << "Waiting for message!" << std::endl;

    int bytesRecv = receiveBuffer(cmdBuffer);

    /*
        TODO:
        Check if bytes received are 5!
    */

    std::cout << "Received Bytes: " << bytesRecv << std::endl;


    if(clientConnected == false){
        if(cmdBuffer[0] != 0x00) return 0;
        else connectClient();
    }

    switch (cmdBuffer[0])
    {

        case CMD_CONNECT:
            clientCommand = CMD_CONNECT;
            break;

        case CMD_STRING:
            clientCommand = CMD_STRING;
            break;


        // Checking if commad is to disconnect.
        case CMD_DISCONNECT:

            clientCommand = CMD_DISCONNECT;
            closeClientSocket();
            return -2;
            break;

        default:
            break;
    }

    std::cout << "What is in the cmdBuffer: " << std::endl;
    for(BYTE item : cmdBuffer){
        std::cout << (int)item << std::endl;
    }
    

    return convertBufferDataSizeToInt(cmdBuffer);

}


int IRCServer::commandAnswer(std::vector<BYTE> replyData){

    BYTE replyCmd = 0x00;
    int replyDataSize = 0;


    if(replyData.empty()) replyDataSize = 0;
    else replyDataSize = replyData.size();

    std::vector<BYTE> dataSizeAsBytes = convertIntToBufferDataSize(replyDataSize);
    replyData.insert(replyData.begin(), dataSizeAsBytes.begin(), dataSizeAsBytes.end());

    switch (clientCommand)
    {
        case CMD_CONNECT:
            if(clientConnected) replyCmd = CMD_CONNECT;
            else replyCmd = 0xFE;
            break;

        case CMD_STRING:
            replyCmd = CMD_STRING;
        
        default:
            break;
    }

    replyData.insert(replyData.begin(), replyCmd);


    std::cout << "Following message will be sent: " << std::endl;
    for(auto item : replyData){
        std::cout << (int)item << std::endl;
    }

    sendBuffer(replyData);
    
    return 0;
}