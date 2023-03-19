/*
 *@(#) Utility.DataChecker.java 0.1 2023/03/17
 *
 * Copyright (c) Omar Ibrahim
 * All rights reserved.
 */
package ibo1.Utility;

import javafx.scene.control.Alert;
import javafx.scene.control.ButtonType;
import javafx.scene.layout.Region;

/**
 * AlertMessage - Class that allows to show information or warnings to the user when exceptions occur or warnings are shown.
 * This class uses JFX to display information outside the nominal usage of the user interface.
 * <p>
 * Implements multiple methods to handle exception and alert messages shown to the user.
 * @author Omar Ibrahim
 * @version 0.1 ( Initial development ).
 */
public abstract class AlertMessage {
    // ////////// //
    // Constants. //
    // ////////// //

    // //////////////// //
    // Class variables. //
    // //////////////// //

    // ////////////// //
    // Class methods. //
    // ////////////// //

    /**
     * Creates an alert where you can define title, header, content and choosing it to only have an okay button
     * and setting the alert Type.
     *
     * @param titleString Title of the alert
     * @param headerString Header of the alert
     * @param contentString Content of the alert
     * @param onlyOkayButton Set need for okay button
     * @param alertType Type of alert of type Alert.AlertType
     */
    public static void showAlert(String titleString, String headerString, String contentString,
                                 boolean onlyOkayButton, Alert.AlertType alertType){

        Alert alert;

        // If no alertType is supplied then set to default value none.
        if(alertType == null) alertType = Alert.AlertType.NONE;

        //Checking if an okay button is needed
        if(onlyOkayButton){
            alert = new Alert(alertType, contentString, ButtonType.OK);

        // ... otherwise set no okay button
        }else{
            alert = new Alert(alertType, contentString);
        }

        alert.setTitle(titleString);
        alert.setHeaderText(headerString);

        // https://stackoverflow.com/questions/28937392/javafx-alerts-and-their-size
        // Making the Alert fit the size of the content text
        alert.getDialogPane( ).setMinHeight( Region.USE_PREF_SIZE );

        // Show alert and halt execution
        alert.showAndWait();
    }

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
     * Creates an alert with an okay button and the supplied information of the parameters
     *
     * @param titleString Title of the alert
     * @param headerString Header of the alert
     * @param contentString Content of the alert
     * @param alertType Type of alert of type Alert.AlertType
     */
    public static void showAlert (String titleString, String headerString, String contentString, Alert.AlertType alertType){
        showAlert(titleString, headerString, contentString, true, alertType);
    }
}
