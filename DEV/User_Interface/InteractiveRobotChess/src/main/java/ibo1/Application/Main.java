/*
 *@(#) Utility.DataChecker.java 0.1 2023/02/28
 *
 * Copyright (c) Omar Ibrahim
 * All rights reserved.
 */
package ibo1.Application;

import Client.IRCClient;
import Protocol.User;
import javafx.application.Application;
import javafx.fxml.FXMLLoader;
import javafx.scene.Parent;
import javafx.scene.Scene;
import javafx.stage.Stage;

import java.io.IOException;

/**
 * Main - Is the starting point of the program. It is the connection point between the backend
 * and the frontend. It will initialize variables and set up the first
 * javafx scene and call it
 * <p>
 * It is used to create an instance of the application by using other class instances.
 * Interactive Robot Chess Server, which is run within a ROS system on a Linux machine.
 * @author Omar Ibrahim
 * @version 0.1 ( Initial development ).
 * @see ibo1.UI.HomePageController
 */
public class Main extends Application {
    // //////////////// //
    //     Constants.   //
    // //////////////// //

    // //////////////// //
    // Class variables. //
    // //////////////// //

    // ////////////// //
    // Class methods. //
    // ////////////// //

    /**
     * JavaFX function to initialize the main Scene
     * and catching IO exception
     *
     * @return The starting Scene
     */
    private Scene initializeMain( ) {

        // Try loading the home page fxml file, and returning it is not null
        try {
            FXMLLoader loader = new FXMLLoader( getClass( ).getResource("../UI/HomePage.fxml") );
            Parent mainLayout = loader.load( );
            return new Scene(mainLayout);
        }
        // Catching Input/Output exception, and showing error message to the user
        catch ( IOException e ) {

            // Displaying error message for the user on GUI
            System.err.println(e.getMessage());
            return null;
        }
    }

    /**
     * The main method launches the program and creates an application,
     * then calls all the methods needed to start the app running.
     * @param args arguments
     */
    public static void main( String[] args ) {
        launch( args );
    }
    // /////////////////// //
    // Instance variables. //
    // /////////////////// //

    private IRCClient ircClient;
    private User user;

    // ///////////// //
    // Constructors. //
    // ///////////// //

    public Main(){
        ircClient = new IRCClient();
    }

    // ////////////////////// //
    // Read/Write properties. //
    // ////////////////////// //


    /**
     * @return returns the IRCClient
     */
    public IRCClient getIrcClient() {
        return ircClient;
    }

    /**
     *
     * @return the current User
     */
    public User getUser() {
        return user;
    }

    /**
     *
     * @param user The user to use
     */
    public void setUser(User user) {
        this.user = user;
    }

    // ///////////////////// //
    // Read-only properties. //
    // ///////////////////// //

    // //////// //
    // Methods. //
    // //////// //

    /**
     * JavaFX function to start the program
     *
     * @param primaryStage  The Primary Stage
     */
    @Override
    public void start( Stage primaryStage ) {

        // Initialize and set scene for the main page
        primaryStage.setTitle( "Interactive Robot Chess" );
        Scene mainScene = initializeMain( );
        primaryStage.setScene( mainScene );

        // Displaying error message if mainScene is null
        if ( mainScene == null ) {
            System.err.println( "the Main Scene is null" );

        // Else display the Home page
        }else{
            primaryStage.show( );
        }
    }
}
