/*
 *@(#) Utility.DataChecker.java 0.1 2023/02/28
 *
 * Copyright (c) Omar Ibrahim
 * All rights reserved.
 */
package ibo1.UI;

import ibo1.Application.Main;
import javafx.fxml.FXML;
import javafx.scene.control.TextField;


/**
 * LoginController - Is the controller for the Login.fxml file, defining which
 * methods the buttons can refer to. This is used after successfully connected to the server.
 * <p>
 * The login function logs in the general user. He will then be referred to the admin or the user panel.
 * @author Omar Ibrahim
 * @version 0.1 ( Initial development ).
 */
public class LoginController {

    // //////////////// //
    //     Constants.   //
    // //////////////// //

    // //////////////// //
    // Class variables. //
    // //////////////// //

    private static Main mainApp;

    // ////////////// //
    // Class methods. //
    // ////////////// //

    @FXML
    private void login( javafx.event.ActionEvent event ) {
        String userName = textFieldUsername.getText();
        String password = textFieldPassword.getText();


    }


    // /////////////////// //
    // Instance variables. //
    // /////////////////// //

    @FXML
    private TextField textFieldUsername;
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
    }

}
