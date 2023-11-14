/*
 *@(#) ibo1.UI.LoginController.java 0.1 2023/03/17
 *
 */
package ibo1.UI;

import ibo1.Client.IRCClient;
import ibo1.CustomException.ProtocolException;
import ibo1.Protocol.User;
import ibo1.Application.Main;
import ibo1.Utility.PopUpMessages;
import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.Node;
import javafx.scene.Parent;
import javafx.scene.Scene;
import javafx.scene.control.Alert;
import javafx.scene.control.TextField;
import ibo1.Enum.ProtocolErrors;
import javafx.stage.Stage;

import java.io.IOException;


/**
 * LoginController - Is the controller for the Login.fxml file, defining which
 * methods the buttons can refer to. This is used after successfully connected to the server.
 * <p>
 * The login function logs in the general user. He will then be referred to the admin or the user panel.
 * The current version only supports a user panel thus an admin will currently also be referred to the
 * user panel.
 *
 * @author Omar Ibrahim
 * @version 0.1 ( Initial development ).
 * @version 1.0 ( Initial release ).
 *
 *
 * @see Main
 * @see IRCClient
 * @see User
 * @see IOException
 * @see ProtocolException
 */
public class LoginController {

    // //////////////// //
    //     Constants.   //
    // //////////////// //

    // //////////////// //
    // Class variables. //
    // //////////////// //

    /**
     * Reference to the main application to access and set global variables.
     */
    private static Main mainApp;


    /**
     * Reference to the ircClient created in main to use for networking
     */
    private static IRCClient ircClient;

    // ////////////// //
    // Class methods. //
    // ////////////// //

    /**
     * This method gets called when the login button is pressed. It starts the login process,
     * with the server and deals with errors.
     *
     * @param event ButtonEvent from JavaFX
     * @throws IOException Thrown by FXMLLoader.load()
     */
    @FXML
    private void login( javafx.event.ActionEvent event ) throws IOException {
        String userName = textFieldUsername.getText();
        String password = textFieldPassword.getText();

        if(userName.isBlank() || password.isBlank()){
            PopUpMessages.showAlert("Fields not filled out",
                    "NOT ALL FIELDS WERE FILLED!",
                    "Please ensure that both user name and password have information within them.",
                    Alert.AlertType.ERROR);
            return;
        }

        // Creating a user object with the username and password
        User user = new User(userName, password);

        try {
            // Try to log in to the system
            user.login(ircClient);
        } catch (IOException e) {
            PopUpMessages.showAlert("IOException", "SENDING OR RECEIVING ERROR",
                    "There was an error with sending or receiving using the client. " +
                                "Possibly lost connection to the server",
                    Alert.AlertType.ERROR);
            return;
        } catch (ProtocolException e) {
            if(e.getMessage().equals(ProtocolErrors.User_DOES_NOTEXIST.toString())){
                PopUpMessages.showAlert("ProtocolError", "USER DOES NOT EXIST!",
                        "The user does not exist.",
                        Alert.AlertType.ERROR);
                return;
            }
            else if(e.getMessage().equals(ProtocolErrors.UNEXPECTED_RETURN_CMD.toString())){
                PopUpMessages.showAlert("ProtocolError", "UNEXPECTED RETURN CMD",
                        "The returned cmdbyte of the protocol was not expected",
                        Alert.AlertType.ERROR);
                return;
            }
            else{
                PopUpMessages.showAlert("ProtocolError", "UNKNOWN ERROR",
                        "Please contact an admin if this error occurs with the specifics on what you did.",
                        Alert.AlertType.ERROR);
                return;
            }
        }

        // Setting the current user.
        mainApp.setUser(user);


        Parent homePageLayout;

        //Checking if the user is an admin
        if(user.isAdmin()){
            // If yes move to admin panel
            // Currently only user panel supported
            System.out.println("Move to Admin panel.");
            homePageLayout = FXMLLoader.load(getClass().getResource("HomePageUser.fxml"));
        }
        // Otherwise the general user is a user, and we move to the user panel
        else{
            System.out.println("Move to User panel.");
            homePageLayout = FXMLLoader.load(getClass().getResource("HomePageUser.fxml"));
        }

        // Setting up the scene switch
        Scene homePageScene = new Scene(homePageLayout);

        // Switching to the defined scene
        Stage stage = (Stage) ((Node) event.getSource()).getScene().getWindow();
        stage.setScene(homePageScene);
        stage.show();
    }

    // /////////////////// //
    // Instance variables. //
    // /////////////////// //

    /**
     * Text field for the username
     */
    @FXML
    private TextField textFieldUsername;

    /**
     * Text field for the password
     */
    @FXML
    private TextField textFieldPassword;

    // //////// //
    // Methods. //
    // //////// //

    /**
     * Used to initialize main and the welcome text.
     */
    @FXML
    public void initialize(){
        mainApp = new Main();
        ircClient = mainApp.getIrcClient();
    }

}
