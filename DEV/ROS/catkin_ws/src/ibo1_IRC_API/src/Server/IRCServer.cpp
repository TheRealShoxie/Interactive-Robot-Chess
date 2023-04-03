#include "ibo1_IRC_API/Server/IRCServer.h"


    // ///////////// //
    // Constructors. //
    // ///////////// //

// Constructor for IRCServer
IRCServer::IRCServer(unsigned int setPort)
    : clientSocket(-1), clientCommand(CMD_CONNECT), port(setPort){
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
int IRCServer::sendBuffer(vector<BYTE> sendingBuffer){
    return send(clientSocket, &sendingBuffer[0], sendingBuffer.size(), 0);
}

//Implementation of Receiving method
int IRCServer::receiveBuffer(vector<BYTE>& receivingBuffer){

    // Waiting for a message
    return recv(clientSocket, &receivingBuffer[0], receivingBuffer.size(), 0);
}


// Converts vector Byte to integer
// https://stackoverflow.com/questions/34943835/convert-four-bytes-to-integer-using-c
int IRCServer::convertBufferDataSizeToInt(vector<BYTE> buffer){

    return int( (buffer[1]) << 24 |
                (buffer[2]) << 16 |
                (buffer[3]) << 8 |
                (buffer[4]));
}

// Converts Int to a vector Byte
vector<BYTE> IRCServer::convertIntToBufferDataSize(int dataSize){
    vector<BYTE> intAsBytes;

    intAsBytes.push_back(dataSize >> 24);
    intAsBytes.push_back(dataSize >> 16);
    intAsBytes.push_back(dataSize >> 8);
    intAsBytes.push_back(dataSize);

    return intAsBytes;
}

    // ////////////////////// //
    // Read/Write properties. //
    // ////////////////////// //

void IRCServer::setClientCommand(BYTE setClientCommand){
    clientCommand = setClientCommand;
}

BYTE IRCServer::getClientCommand(){
    return clientCommand;
}




    // ///////////////////// //
    // Read-only properties. //
    // ///////////////////// //


unsigned int IRCServer::getPort(){
    return port;
}

int IRCServer::getClientSocket(){
    return clientSocket;
}

bool IRCServer::getClientConnected(){
    return clientConnected;
}


    // //////// //
    // Methods. //
    // //////// //


// Implementation of initiateServerSocket
void IRCServer::initiateServerSocket(){
    // Creating server socket
    int listeningSocket = socket(AF_INET, SOCK_STREAM, 0);
    if (listeningSocket == -1) throw runtime_error("Couldn't initialize socket!");

    cout << "Starting...." << endl;

    // Binding the socket to a IP Address and port
    sockaddr_in hint;
    hint.sin_family = AF_INET;
    // htons = host to network host
    hint.sin_port = htons(port);
    // inet_pton converts a pointer string of number to an array of integers
    // We use "0.0.0.0" for any address
    inet_pton(AF_INET, "0.0.0.0", &hint.sin_addr);

    if(bind(listeningSocket, (sockaddr *)&hint, sizeof(hint)) == -1) throw runtime_error("Couldn't bin Socket to IP Address or Port!");


    // Marking the socket for listening in
    if(listen(listeningSocket, SOMAXCONN) == -1) throw runtime_error("Cannot listen!");


    // Accepting a client
    sockaddr_in client;
    socklen_t clientSize = sizeof(client);
    char host[NI_MAXHOST];
    char service[NI_MAXSERV];

    // Accepting an incoming client
    clientSocket = accept(listeningSocket, 
                          (sockaddr *)&client,
                          &clientSize);
            
    if (clientSocket == -1) throw runtime_error("Problem with client connecting!");
}

// Gets the data from the input buffer as a vector and returns that.
vector<BYTE> IRCServer::getData(int bufferSize){
    vector<BYTE> dataBuffer;
    dataBuffer.resize(bufferSize);
    int bytesRecv = receiveBuffer(dataBuffer);

    return dataBuffer;
}


// Extracts the command and returns the size of the data
/*
    * returns number of bytes to read
    * returns -1 when client not yet connected!
    * returns -2 if told to disconnect
    * returns -3 if cmd doesnt exist
    * returns -4 if bytes rescv arent length of proctocol(5)
*/
int IRCServer::commandExtraction(){


    vector<BYTE> cmdBuffer;
    cmdBuffer.resize(5);

    int bytesRecv = receiveBuffer(cmdBuffer);
    cout << "Received Bytes: " << bytesRecv << endl;
    cout << "Cmd Byte: " << (int)cmdBuffer[0] << endl;

    if(bytesRecv != 5){
        return -4;
    }


    if(clientConnected == false){
        if(cmdBuffer[0] != CMD_CONNECT){
            clientCommand = ERROR_CONNECT;
            return -1;
        }
        else connectClient();
    }

    switch (cmdBuffer[0])
    {

        case CMD_CONNECT:
            clientCommand = CMD_CONNECT;
            break;

        case CMD_LOGIN:
            clientCommand = CMD_LOGIN;
            break;

        case CMD_CREATEUSER:
            clientCommand = CMD_CREATEUSER;
            break;

        case CMD_GETCHESSENGINES:
            clientCommand = CMD_GETCHESSENGINES;
            break;

        case CMD_STARTCHESSENGINE:
            clientCommand = CMD_STARTCHESSENGINE;
            break;

        case CMD_STOPCHESSENGINE:
            clientCommand = CMD_STOPCHESSENGINE;
            break;

        case CMD_GETCHESSENGINEOPTIONS:
            clientCommand = CMD_GETCHESSENGINEOPTIONS;
            break;

        case CMD_SETCHESSENGINEOPTIONS:
            clientCommand = CMD_SETCHESSENGINEOPTIONS;
            break;

        case CMD_PLAYERMOVE:
            clientCommand = CMD_PLAYERMOVE;
            break;

        case CMD_CHESSENGINEMOVE:
            clientCommand = CMD_CHESSENGINEMOVE;
            break;

        case CMD_SETSEARCHOPTIONS:
            clientCommand = CMD_SETSEARCHOPTIONS;
            break;

        // Checking if command is to disconnect.
        case CMD_DISCONNECT:

            clientCommand = CMD_DISCONNECT;
            closeClientSocket();
            return -2;
            break;

        default:
            clientCommand = ERROR_CMD_UNRECOGNIZABLE;
            return -3;
            break;
    }

    return convertBufferDataSizeToInt(cmdBuffer);

}


// Used to send an answer to the client.
int IRCServer::sendAnswer(vector<BYTE> replyData){

    BYTE replyCmd = 0x00;
    int replyDataSize = 0;


    if(replyData.empty()) replyDataSize = 0;
    else replyDataSize = replyData.size();

    vector<BYTE> dataSizeAsBytes = convertIntToBufferDataSize(replyDataSize);
    replyData.insert(replyData.begin(), dataSizeAsBytes.begin(), dataSizeAsBytes.end());

    replyCmd = clientCommand;

    replyData.insert(replyData.begin(), replyCmd);

    sendBuffer(replyData);
    
    return 0;
}