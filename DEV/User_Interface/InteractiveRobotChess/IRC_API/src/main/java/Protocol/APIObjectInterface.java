/*
 *@(#) Utility.DataChecker.java 0.1 2023/02/28
 *
 * Copyright (c) Omar Ibrahim
 * All rights reserved.
 */
package Protocol;

import Client.IRCClient;

/**
 *
 * <p>
 * @author Omar Ibrahim
 * @version 0.1 ( Initial development ).
 **/
public interface APIObjectInterface<T> {

    // ////////// //
    // Constants. //
    // ////////// //

    // //////////////// //
    // Class variables. //
    // //////////////// //

    // ////////////// //
    // Class methods. //
    // ////////////// //

    public byte[] parseToByteArray();

    // /////////////////// //
    // Instance variables. //
    // /////////////////// //

    // ///////////// //
    // Constructors. //
    // ///////////// //

    // ////////////////////// //
    // Read/Write properties. //
    // ////////////////////// //

    // ///////////////////// //
    // Read-only properties. //
    // ///////////////////// //

    // //////// //
    // Methods. //
    // //////// //

    void updateValue(IRCClient ircClient);
}
