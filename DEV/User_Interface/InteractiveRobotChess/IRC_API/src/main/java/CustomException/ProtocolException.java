/*
 *@(#) Utility.DataChecker.java 0.1 2023/03/17
 *
 * Copyright (c) Omar Ibrahim
 * All rights reserved.
 */
package CustomException;

/**
 * ClassName - ClassDescription initial
 * <p>
 * What it does
 * @author Omar Ibrahim
 * @version 0.1 ( Initial development ).
 * @see Exception
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
