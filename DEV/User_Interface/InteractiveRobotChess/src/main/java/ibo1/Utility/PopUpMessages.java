/*
 *@(#) Utility.DataChecker.java 0.1 2023/03/17
 *
 * Copyright (c) Omar Ibrahim
 * All rights reserved.
 */
package ibo1.Utility;

import javafx.scene.control.Alert;
import javafx.scene.control.ButtonType;
import javafx.scene.control.ChoiceDialog;
import javafx.scene.layout.Region;

import java.util.List;
import java.util.Optional;

/**
 * AlertMessage - Class that allows to show information or warnings to the user when exceptions occur or warnings are shown.
 * This class uses JFX to display information outside the nominal usage of the user interface.
 * <p>
 * Implements multiple methods to handle exception and alert messages shown to the user.
 * @author Omar Ibrahim
 * @version 0.1 ( Initial development ).
 */
public abstract class PopUpMessages {
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
    private static void createAndShowAlert(String titleString, String headerString, String contentString,
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


    /**
     * Creates and shows a choice dialog.
     * <p>
     * Waits for the selection of a choice and returns that choice.
     *
     * @param titleString Title text of the choice dialog
     * @param headerString Header text of the choice dialog
     * @param contentString Content text of the choice dialog
     * @param choices Choices of the ChoiceDialog of type List<String>
     * @return returns the selected choice as String
     */
    private static String createAndShowChoiceDialog(String titleString, String headerString, String contentString, List<String> choices){

        //Creating a new ChoiceDialog and adding the choices
        ChoiceDialog choiceDialog = new ChoiceDialog(choices.get(0), choices);

        // Setting the title, header and content text
        choiceDialog.setTitle(titleString);
        choiceDialog.setHeaderText(headerString);
        choiceDialog.setContentText(contentString);

        // Showing and waiting till a user selected and confirmed
        Optional<String> result = choiceDialog.showAndWait();

        // Returns the selection
        return result.get();
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
        createAndShowAlert(titleString, headerString, contentString, true, alertType);
    }


    /**
     * Creates a choice dialog for chess engines.
     * <p>
     * This is used to select which chess engine will be used.
     *
     * @param choices Possible chess engines as String
     * @return the selected chess engine
     */
    public static String showChoiceDialogChessEngine(List<String> choices){
        return createAndShowChoiceDialog("Chess Engine Picker","Select the chess engine to use:","ChessEngine:", choices);
    }

    public static String showChoiceDialogPromotion(List<String> choices){
        return createAndShowChoiceDialog("Promotion picker","Select the chess piece to promote into:","Chess Piece:", choices);
    }
}
