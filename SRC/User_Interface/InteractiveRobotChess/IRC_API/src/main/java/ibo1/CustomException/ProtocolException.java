/*
 *@(#) ibo1.CustomException.ProtocolException.java 0.1 2023/04/30
 *
 */
package ibo1.CustomException;

/**
 * ProtocolException - Custom Exception used for Protocol errors
 *
 * @author Omar Ibrahim
 * @version 0.1 ( Initial development ).
 * @version 1.0 ( Initial release ).
 * @see ibo1.Protocol.User
 * @see ibo1.Protocol.ChessEngine
 * @see ibo1.ChessSpecific.ChessBoard
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
