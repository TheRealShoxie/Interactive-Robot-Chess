/*
 *@(#) Utility.DataChecker.java 0.1 2023/02/28
 *
 * Copyright (c) Omar Ibrahim
 * All rights reserved.
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
 * @author Omar Ibrahim
 * @version 0.1 ( Initial development ).
 * @see DataChecker
 * @see ProtocolObject
 * @see InvalidDataException
 * @see Protocol.APIObjectInterface
 */
public class IRCClient {

    // ////////// //
    // Constants. //
    // ////////// //

    // //////////////// //
    // Class variables. //
    // //////////////// //

    private Socket clientSocket;
    private OutputStream out;
    private InputStream in;


    // ////////////// //
    // Class methods. //
    // ////////////// //

    // /////////////////// //
    // Instance variables. //
    // /////////////////// //

    private String ipAddress;
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
        setIpAddress(ipAddress);
        setPort(port);
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
     * @param sendData Protocol Object to be sent.
     * @throws IOException thrown by OutputStream write
     */
    public void send(ProtocolObject sendData) throws IOException {
        out.write(sendData.toByteArray());
    }

    /**
     * Receives data from the clientSocket as a byte array and returns a protocol object.
     *
     * @return returns the read buffer as a byte array
     * @throws IOException thrown by InputStream read
     */
    public ProtocolObject receive() throws IOException {

        ProtocolObject protocolObject = new ProtocolObject();

        protocolObject.setCmdByte((byte) in.read());
        protocolObject.setDataSize(in.readNBytes(4));
        protocolObject.setData(in.readNBytes(protocolObject.getDataSize()));

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

        ProtocolObject sendOpenConnection = new ProtocolObject();
        sendOpenConnection.setCmdByte((byte) 0x00);
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

        //TODO: Needs to be rewritten! Both on C++ side and on Java side.

        ProtocolObject sendCloseConnection = new ProtocolObject();
        sendCloseConnection.setCmdByte((byte) 0xFF);
        sendCloseConnection.setDataSize(0);

        send(sendCloseConnection);

        ProtocolObject receivingData = receive();

        System.out.println(receivingData);

        in.close();
        out.close();
        clientSocket.close();
    }
}
