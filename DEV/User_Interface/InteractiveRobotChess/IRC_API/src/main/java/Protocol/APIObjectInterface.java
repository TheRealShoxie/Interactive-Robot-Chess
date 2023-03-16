/*
 *@(#) Utility.DataChecker.java 0.1 2023/02/28
 *
 * Copyright (c) Omar Ibrahim
 * All rights reserved.
 */
package Protocol;

import Client.IRCClient;

/**
 * DataInterface - Interface on what classes Data objects which can be sent and received need to consist of
 * <p>
 * This abstract class handles checking data, and throwing an exception if data is invalid or incorrectly formatted.
 * If data is invalid it throws an exception.
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

    private byte[] parseToByteArray() {
        return new byte[0];
    }

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
