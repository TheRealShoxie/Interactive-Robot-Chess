/*
 *@(#) Utility.DataChecker.java 0.1 2023/02/28
 *
 * Copyright (c) Omar Ibrahim
 * All rights reserved.
 */
package Protocol;

import Client.IRCClient;

/**
 * User - Object Representation for a User or Admin.
 * <p>
 * This class handles all the requests sent to the server to get information from the server regarding a User.
 * @author Omar Ibrahim
 * @version 0.1 ( Initial development ).
 **/
public class User implements APIObjectInterface{

    // ////////// //
    // Constants. //
    // ////////// //

    // //////////////// //
    // Class variables. //
    // //////////////// //

    // ////////////// //
    // Class methods. //
    // ////////////// //

    // /////////////////// //
    // Instance variables. //
    // /////////////////// //

    private String username;
    private String password;
    private boolean isAdmin;

    // ///////////// //
    // Constructors. //
    // ///////////// //

    public User(String username, String password){
        this.username = username;
        this.password = password;
    }

    // ////////////////////// //
    // Read/Write properties. //
    // ////////////////////// //

    // ///////////////////// //
    // Read-only properties. //
    // ///////////////////// //

    // //////// //
    // Methods. //
    // //////// //
    @Override
    public void updateValue(IRCClient ircClient) {

    }
}
