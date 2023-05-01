/*
 *@(#) Protocol.User.java 0.1 2023/03/17
 *
 * Copyright (c) Omar Ibrahim
 * All rights reserved.
 */
package Protocol;

import Client.IRCClient;
import CustomException.ProtocolException;
import Enum.ProtocolErrors;

import java.io.IOException;
import java.nio.charset.StandardCharsets;

/**
 * User - Object Representation for a User or Admin.
 * <p>
 * This class handles all the requests sent to the server to get information from the server regarding a User.
 * @author Omar Ibrahim
 * @version 0.1 ( Initial development ).
 * @version 1.0 ( Initial release ).
 * @see IRCClient
 * @see ProtocolException
 * @see ProtocolErrors
 * @see ProtocolObject
 **/
public class User{

    // ////////// //
    // Constants. //
    // ////////// //

    /**
     * Cmd byte from the protocol
     */
    final static byte cmdByteLogin = (byte)0x02;

    /**
     * Error byte from the protocol
     */
    final static byte userDoesNotExist = (byte)0xFC;

    /**
     * Error byte from the protocol
     */
    final static byte unrecognizableCmd = (byte)0xFE;

    // //////////////// //
    // Class variables. //
    // //////////////// //

    // ////////////// //
    // Class methods. //
    // ////////////// //

    /**
     * Converts the Object to a ProtocolObject to use in the client.
     *
     * @return the protocol object for User
     */
    private ProtocolObject toProtocolObject(){
        String dataString = username + "\u241f" +password;//"\u2400" +password;

        ProtocolObject protocolObject = new ProtocolObject();
        protocolObject.setCmdByte(cmdByteLogin);

        // Converting to UTF8
        //byte[] dataStringAsBytes = dataString.getBytes(StandardCharsets.UTF_8);
        //protocolObject.setDataSize(dataStringAsBytes.length);
        protocolObject.setData(dataString.getBytes(StandardCharsets.UTF_8));

        return protocolObject;
    }

    /**
     * Converts a byte to a boolean.
     * <p>
     * If byte represents 0x00 then it is false otherwise true.
     * @param inputByte the byte to be converted
     * @return boolean value of the byte
     */
    private boolean byteToBoolean(byte inputByte){
        return inputByte != 0x00;
    }

    // /////////////////// //
    // Instance variables. //
    // /////////////////// //

    /**
     * Username of the user
     */
    private String username;


    /**
     * Password of the user
     */
    private String password;


    /**
     * Is the user an admin
     */
    private boolean isAdmin;

    // ///////////// //
    // Constructors. //
    // ///////////// //

    /**
     * Constructor for User
     *
     * @param username username
     * @param password password
     */
    public User(String username, String password){
        this.username = username;
        this.password = password;
    }

    // ////////////////////// //
    // Read/Write properties. //
    // ////////////////////// //


    /**
     * @param admin if the general user is an admin
     */
    public void setAdmin(boolean admin) {
        isAdmin = admin;
    }

    /**
     * @return returns the username
     */
    public String getUsername() {
        return username;
    }

    /**
     * @return returns the password
     */
    public String getPassword() {
        return password;
    }

    /**
     * @return returns boolean for general user is admin or normal user
     */
    public boolean isAdmin() {
        return isAdmin;
    }

    // ///////////////////// //
    // Read-only properties. //
    // ///////////////////// //

    // //////// //
    // Methods. //
    // //////// //

    /**
     * Logs in to the System
     * <p>
     * Throws ProtocolException for unexpected returned cmdByte. Otherwise, it sets if user is admin or user.-
     * @param ircClient ircClient to be used
     * @throws IOException thrown by sending and receiving the buffer
     * @throws ProtocolException thrown if user doesn't exist or returned cmd does not match expected
     */
    public void login(IRCClient ircClient) throws IOException, ProtocolException {
        ircClient.send(toProtocolObject());

        ProtocolObject receivedProtocol = ircClient.receive();

        byte receivedCommandByte = receivedProtocol.getCmdByte();

        // Setting admin
        if(receivedCommandByte == cmdByteLogin) setAdmin(byteToBoolean(receivedProtocol.getData()[0]));
        // Checking if we received User does not exist error
        else if(receivedCommandByte == userDoesNotExist) throw new ProtocolException(ProtocolErrors.User_DOES_NOTEXIST.toString());
        // Command was not recognizable
        else if(receivedCommandByte == unrecognizableCmd) throw new ProtocolException(ProtocolErrors.UNRECOGNIZABLE_CMD.toString());
        // Does not match expected return
        else throw new ProtocolException(ProtocolErrors.UNEXPECTED_RETURN_CMD.toString());

    }

}
