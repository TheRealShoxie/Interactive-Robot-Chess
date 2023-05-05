/*
 *@(#) Client.IRCClient.java 0.1 2023/04/30
 *
 */
package Client;

import CustomException.InvalidDataException;
import Utility.DataChecker;
import Protocol.ProtocolObject;


import java.io.*;
import java.net.Socket;

/**
 * IRCClient - Interactive Robot Chess Client class is responsible for the communication with the
 * Interactive Robot Chess Server, which is run within a ROS system on a Linux machine.
 *
 * @author Omar Ibrahim
 * @version 0.1 ( Initial development ).
 * @version 1.0 ( Initial release ).
 *
 * @see DataChecker
 * @see ProtocolObject
 * @see InvalidDataException
 * @see Protocol.ChessEngine
 * @see ChessSpecific.ChessBoard
 * @see Protocol.User
 */
public class IRCClient {

    // ////////// //
    // Constants. //
    // ////////// //

    // //////////////// //
    // Class variables. //
    // //////////////// //

    /**
     * Socket of the client
     */
    private Socket clientSocket;


    /**
     * Output stream to write the protocols to
     */
    private OutputStream out;


    /**
     * Input stream to read the protocols from
     */
    private InputStream in;

    // ////////////// //
    // Class methods. //
    // ////////////// //

    // /////////////////// //
    // Instance variables. //
    // /////////////////// //

    /**
     * IP address of the server to communicate with
     */
    private String ipAddress;


    /**
     * Port of the server to communicate with
     */
    private int port;

    // ///////////// //
    // Constructors. //
    // ///////////// //

    /**
     * Creates a default Constructor where ip address and port are predefined and set.
     */
    public IRCClient(){
        ipAddress = "127.0.0.1";
        port = 54001;
    }

    /**
     * Creates a new Client.IRCClient
     *
     * @param ipAddress IPv4 address to connect to
     * @param port port to connect to
     * @throws InvalidDataException thrown by invalid IPv4 address
     */
    public IRCClient(String ipAddress, int port) throws InvalidDataException {
        setIpAddress(ipAddress);
        setPort(port);
    }

    /**
     * Creates a new Client.IRCClient and connects automatically
     *
     * @param ipAddress IPv4 address to connect to
     * @param port port to connect to
     * @param autoConnect automatically connects to the server
     * @throws IOException thrown by connecting to server
     * @throws InvalidDataException thrown by invalid IPv4 address
     */
    public IRCClient(String ipAddress, int port, boolean autoConnect) throws IOException, InvalidDataException {

        // Set the ip Address
        setIpAddress(ipAddress);

        // Set the port
        setPort(port);

        // Checking if we should auto connect. If yes open connection
        if(autoConnect) openConnection();
    }

    // ////////////////////// //
    // Read/Write properties. //
    // ////////////////////// //

    /**
     * Read ipAddress property.
     *
     * @return ipAddress
     */
    public String getIpAddress() {
        return ipAddress;
    }

    /**
     * Read port property.
     *
     * @return port
     */
    public int getPort() {
        return port;
    }

    /**
     * Sets ipAddress property. Further it checks if ipAddress is correctly formatted.
     *
     * @param ipAddress IPv4 address to connect to
     * @throws InvalidDataException thrown by invalid IPv4 address
     */
    public void setIpAddress(String ipAddress) throws InvalidDataException {
        this.ipAddress = DataChecker.checkValidIPAddress(ipAddress);
    }

    /**
     * Sets port property.
     *
     * @param port port to connect to
     */
    public void setPort(int port) {
        this.port = port;
    }

    // ///////////////////// //
    // Read-only properties. //
    // ///////////////////// //

    // //////// //
    // Methods. //
    // //////// //

    /**
     * Sends data to the clientSocket as a byte array.
     *
     * @param sendData Protocol Object to be sent.
     * @throws IOException thrown by OutputStream write
     */
    public void send(ProtocolObject sendData) throws IOException {

        System.out.println("----------\nI am sending following to the server: " +sendData.toString() +"\n----------\n");

        // Sends supplied  ProtocolObject.toByteArray()
        out.write(sendData.toByteArray());
    }

    /**
     * Receives data from the clientSocket as a byte array and returns a protocol object.
     *
     * Current version has no timeout build in. This needs to be expanded on in the future.
     *
     * @return returns the read buffer as a byte array
     * @throws IOException thrown by InputStream read
     */
    public ProtocolObject receive() throws IOException {

        // Create a new ProtocolObjects
        ProtocolObject protocolObject = new ProtocolObject();

        // Read the first byte and set that as the cmd byte
        protocolObject.setCmdByte((byte) in.read());

        // Read the next 4 bytes which represent the DataSize and set them to the protocolObject
        protocolObject.setDataSize(in.readNBytes(4));

        // Read n amounts of bytes specified by the dataSize from the previous 4 bytes
        protocolObject.setData(in.readNBytes(protocolObject.getDataSize()));

        // Printing what I received on the socked
        System.out.println("----------\nI received following on the socket: \n" +protocolObject.toString() +"\n----------\n");

        // Return the protocol
        return protocolObject;
    }

    /**
     * Tries to connect to the IRCServer.
     *
     * @throws java.net.UnknownHostException throws an UnknownHostException
     * @throws java.io.IOException throws an IOException
     */
    public void openConnection()
            throws java.net.UnknownHostException, java.io.IOException{

        // Creating client socket
        clientSocket = new Socket(getIpAddress(), getPort());
        out = clientSocket.getOutputStream();
        in = clientSocket.getInputStream();

        // Set up the sendOpen Connection protocol and send it
        ProtocolObject sendOpenConnection = new ProtocolObject();
        sendOpenConnection.setCmdByte((byte) 0x01);
        sendOpenConnection.setDataSize(0);

        // Sending connecting command to server
        send(sendOpenConnection);

        // Receiving answer from the server
        ProtocolObject receivingData = receive();

        // Printing received message
        System.out.println(receivingData);
    }

    /**
     * Tries to disconnect from the IRCServer.
     *
     * @throws java.io.IOException throws an IOException
     */
    public void closeConnection() throws java.io.IOException{

        //Set up the disconnect protocol objects
        ProtocolObject sendCloseConnection = new ProtocolObject();
        sendCloseConnection.setCmdByte((byte) 0x00);
        sendCloseConnection.setDataSize(0);


        // Send the disconnect protocol
        send(sendCloseConnection);

        // Close the reader and writer and the socket
        in.close();
        out.close();
        clientSocket.close();
    }
}