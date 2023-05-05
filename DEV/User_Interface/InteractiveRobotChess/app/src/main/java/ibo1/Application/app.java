/*
 *@(#) ibo1.Application.app.java 0.1 2023/05/01
 *
 */

package ibo1.Application;

/**
 * app - Is the starting point of the program.
 * It calls the Main.startUserInterface() function to start the user interface.
 *
 *
 * @author Omar Ibrahim
 * @version 0.1 ( Initial development ).
 * @version 1.0 ( Initial release ).
 * @see Main
 */
public class app {
    // //////////////// //
    //     Constants.   //
    // //////////////// //

    // ///////////// //
    // Constructors. //
    // ///////////// //

    // //////////////// //
    // Class variables. //
    // //////////////// //

    // ////////////// //
    // Class methods. //
    // ////////////// //

    // /////////////////// //
    // Instance variables. //
    // /////////////////// //

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
     * The main method launches the program,
     * it calls Main.startUserInterface and passes the args to start the JavaFX environment
     * @param args arguments passed to the program
     */
    public static void main(String[] args) {
        Main.startUserInterface(args);
    }

}
