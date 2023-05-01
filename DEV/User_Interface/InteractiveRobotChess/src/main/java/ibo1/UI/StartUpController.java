/*
 *@(#) UI.StartUpController.java 0.1 2023/03/17
 *
 * Copyright (c) Omar Ibrahim
 * All rights reserved.
 */
package ibo1.UI;


import Client.IRCClient;
import CustomException.InvalidDataException;
import ibo1.Application.Main;
import ibo1.Utility.PopUpMessages;
import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.Node;
import javafx.scene.Parent;
import javafx.scene.Scene;
import javafx.scene.control.Alert;
import javafx.scene.control.Label;
import javafx.scene.control.TextField;
import javafx.stage.Stage;

import java.io.IOException;

/**
 * StartUpController - Is the controller for the StartUp.fxml file, defining which
 * methods the buttons can refer to. This is the starting point of the User Interface,
 * and extends to the rest of the User Interface files.
 * <p>
 * It is used to call methods depending on which buttons are pressed on the home page.
 * @author Omar Ibrahim
 * @version 0.1 ( Initial development ).
 * @version 1.0 ( Initial release ).
 *
 * @see PopUpMessages
 * @see Main
 * @see IRCClient
 * @see IOException
 * @see InvalidDataException
 */
public class StartUpController {

    // //////////////// //
    //     Constants.   //
    // //////////////// //

    // //////////////// //
    // Class variables. //
    // //////////////// //


    /**
     * Welcome text to display on the labelWelcome javaFX Label
     */
    private static final String labelWelcomeText = "Welcome to the Interactive Robot Chess User Interface!";

    /**
     * Reference to the ircClient created in main to use for networking
     */
    private static IRCClient ircClient;

    // ////////////// //
    // Class methods. //
    // ////////////// //

    /**
     * Connects to the server and when connection is successful switches to the
     * Login scene.
     * @param event Used to switch to Log-in scene.
     * @throws IOException Thrown if Login scene cannot be loaded.
     */
    @FXML
    private void connectToServer( javafx.event.ActionEvent event ) throws IOException{


        //Tries to set IP address and connect to the server
        try {
            ircClient.setIpAddress(textFieldIPAddress.getText());
            ircClient.openConnection();

            //Exception if IP Address is invalid
        } catch (InvalidDataException e) {
            PopUpMessages.showAlert("InvalidDataException", "INVALID IP ADDRESS!",
                    "The entered IP address is invalid. Please refer to documentation on what format " +
                            "a IP address is made of.", Alert.AlertType.ERROR);
            return;

            //Exception if Connection not possible or refused.
        } catch(IOException e ){
            PopUpMessages.showAlert("IOException", "CONNECTION REFUSED!",
                    "The connection to the server could not be made. Please ensure it is connected " +
                            "correctly and can be called. Also double check the supplied IP Address.",
                    Alert.AlertType.ERROR);
            return;
        }

        //If connection to the server is successful then switch to the login scene.
        Parent loginSceneLayout = FXMLLoader.load(getClass().getResource("Login.fxml"));

        Scene loginScene = new Scene(loginSceneLayout);

        Stage stage = (Stage) ((Node) event.getSource()).getScene().getWindow();
        stage.setScene(loginScene);
        stage.show();
    }

    // /////////////////// //
    // Instance variables. //
    // /////////////////// //

    /**
     * Reference to the JavaFX Label: labelWelcome
     */
    @FXML
    private Label labelWelcome;

    /**
     * Reference to the JavaFX Text field for the to connect to ip address.
     */
    @FXML
    private TextField textFieldIPAddress;


    // //////// //
    // Methods. //
    // //////// //


    /**
     * Used to initialize main and the welcome text.
     */
    @FXML
    public void initialize(){
        Main mainApp = new Main();
        ircClient = mainApp.getIrcClient();
        labelWelcome.setText(labelWelcomeText);
    }

}
