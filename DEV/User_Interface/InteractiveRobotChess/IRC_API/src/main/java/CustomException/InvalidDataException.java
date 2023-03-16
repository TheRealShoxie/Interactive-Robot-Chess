/*
 *@(#) CustomException.InvalidDataException.java 0.1 2023/02/28
 *
 * Copyright (c) Omar Ibrahim
 * All rights reserved.
 */
package CustomException;

import Utility.DataChecker;

/**
 * InvalidDataException - Exception for invalid data
 * <p>
 * This class enables to throw InvalidDataException for the Utility.DataChecker.
 * @author Omar Ibrahim
 * @version 0.1 ( Initial development ).
 * @see DataChecker
 * @see Exception
 */
public class InvalidDataException extends Exception {
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

    /**
     * Custom exception for invalid Data
     *
     * @param message
     */
    // //////// //
    // Methods. //
    // //////// //
    public InvalidDataException(String message){
        super(message);
    }
}
