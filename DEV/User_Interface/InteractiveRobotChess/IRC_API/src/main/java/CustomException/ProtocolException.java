/*
 *@(#) CustomException.ProtocolException.java 0.1 2023/04/30
 *
 * Copyright (c) Omar Ibrahim
 * All rights reserved.
 */
package CustomException;

/**
 * ProtocolException - Custom Exception used for Protocol errors
 *
 * @author Omar Ibrahim
 * @version 0.1 ( Initial development ).
 * @version 1.0 ( Initial release ).
 * @see Protocol.User
 * @see Protocol.ChessEngine
 * @see ChessSpecific.ChessBoard
 */
public class ProtocolException extends Exception {
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

    /**
     * Custom Exception for Protocol Exceptions
     *
     * @param message used for the exception
     */
    public ProtocolException(String message){
        super(message);
    }
}
