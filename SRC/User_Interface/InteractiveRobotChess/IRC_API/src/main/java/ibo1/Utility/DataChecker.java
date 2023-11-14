/*
 *@(#) ibo1.Utility.DataChecker.java 0.1 2023/02/28
 *
 */
package ibo1.Utility;

import ibo1.CustomException.InvalidDataException;

import java.util.regex.Matcher;
import java.util.regex.Pattern;

/**
 * DataChecker - Checking data, making sure that data is valid.
 * <p>
 * This abstract class handles checking data, and throwing an exception if data is invalid or incorrectly formatted.
 * If data is invalid it throws an exception.
 * @author Omar Ibrahim
 * @version 0.1 ( Initial development ).
 * @version 1.0 ( Initial release ).
 * @see InvalidDataException
 * @see ibo1.Client.IRCClient
 **/
public abstract class DataChecker {
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
     * Takes a string and checks if it is an IP Address.
     *
     * @param ipAddress ipAddress of the IRC Server.
     * @return Returns input if valid
     * @throws InvalidDataException Thrown if input is not an IP Address.
     */
    public static String checkValidIPAddress(String ipAddress) throws InvalidDataException {
        // Setup REGEX to check for valid formatting (tested on https://regexr.com/)
        Pattern validFormat = Pattern.compile("(([0-9]{1,4}).[0-9]{1,4}.[0-9]{1,4}.[0-9]{1,4})");

        // Check if the input matches the pattern
        Matcher checkFormat = validFormat.matcher(ipAddress);

        // If it matches to format, pass back the string...
        if (checkFormat.matches()) return ipAddress;

        // ...or an error is thrown
        throw new InvalidDataException("IP Address not correctly formatted!");
    }
}

