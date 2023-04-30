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
 * @version 0.2 ( Second development ).
 * @version 1.0 ( Initial release ).
 *
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

    // //////// //
    // Methods. //
    // //////// //

    /**
     * Custom exception for invalid Data
     *
     * @param message used for the exception
     */
    public InvalidDataException(String message){
        super(message);
    }
}
