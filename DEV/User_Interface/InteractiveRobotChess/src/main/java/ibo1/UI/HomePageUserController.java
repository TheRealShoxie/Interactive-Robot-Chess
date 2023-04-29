/*
 *@(#) Utility.DataChecker.java 0.1 2023/03/17
 *
 * Copyright (c) Omar Ibrahim
 * All rights reserved.
 */
package ibo1.UI;

import Client.IRCClient;
import CustomException.ProtocolException;
import Protocol.ChessEngine;
import ibo1.Application.Main;
import ibo1.Utility.PopUpMessages;
import javafx.fxml.FXML;
import javafx.scene.Node;
import javafx.scene.control.Alert;
import Enum.ProtocolErrors;
import javafx.scene.layout.*;
import java.io.IOException;
import java.util.List;

/**
 * HomePageUserController - Is the controller for the HomePageUser.fxml file, defining which
 * methods the buttons can refer to. This is the general Scene the user will be interacting with.
 * <p>
 * It is used to call methods depending on which buttons are pressed on the home page for the user.
 * @author Omar Ibrahim
 * @version 0.1 ( Initial development ).
 */
public class HomePageUserController {
    // //////////////// //
    //     Constants.   //
    // //////////////// //

    // //////////////// //
    // Class variables. //
    // //////////////// //

    private static Main mainApp;
    private static IRCClient ircClient;
    private static ChessEngine chessEngine;

    // ////////////// //
    // Class methods. //
    // ////////////// //

    @FXML
    private void startChessGame( javafx.event.ActionEvent event ){

        // Start selection of the chess Engine
        String selectedSimState = PopUpMessages.showChoiceDialogChessEngine(chessEngine.getSimChoices());

        boolean setSimulation = false;

        try{
            setSimulation = chessEngine.setSimulation(ircClient, selectedSimState);
        } catch (IOException e){
            PopUpMessages.showAlert("IOException", "SENDING OR RECEIVING ERROR",
                    "There was an error with sending or receiving using the client. " +
                            "Possibly lost connection to the server",
                    Alert.AlertType.ERROR);
            return;
        } catch (ProtocolException e) {
            if(e.getMessage().equals(ProtocolErrors.UNEXPECTED_RETURN_CMD.toString())){
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


        if(!setSimulation){
            PopUpMessages.showAlert("Sim State", "Sim State Error",
                    "Sim state was not set please contact an admin.",
                    Alert.AlertType.ERROR);
            return;
        }

        // Getting all possible chess engines that can be selected
        try{
            chessEngine.getPossibleChessEngines(ircClient);
        } catch (IOException e) {
            PopUpMessages.showAlert("IOException", "SENDING OR RECEIVING ERROR",
                    "There was an error with sending or receiving using the client. " +
                            "Possibly lost connection to the server",
                    Alert.AlertType.ERROR);
            return;
        } catch (ProtocolException e) {
            if(e.getMessage().equals(ProtocolErrors.UNEXPECTED_RETURN_CMD.toString())){
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

        // Start selection of the chess Engine
        String selectedChessEngine = PopUpMessages.showChoiceDialogChessEngine(chessEngine.getChessEngineChoices());


        try{
            chessEngine.startChessEngine(ircClient, selectedChessEngine);
        } catch (IOException e) {
            PopUpMessages.showAlert("IOException", "SENDING OR RECEIVING ERROR",
                    "There was an error with sending or receiving using the client. " +
                            "Possibly lost connection to the server",
                    Alert.AlertType.ERROR);
            return;
        } catch (ProtocolException e) {
            if(e.getMessage().equals(ProtocolErrors.CHESSENGINE_NOT_FOUND.toString())){
                PopUpMessages.showAlert("ProtocolError", "CHESS ENGINE NOT FOUND!",
                        "The selected chess engine could not be found.",
                        Alert.AlertType.ERROR);
                return;
            }
            else if(e.getMessage().equals(ProtocolErrors.CHESSENGINE_NOT_STARTED.toString())){
                PopUpMessages.showAlert("ProtocolError", "CHESS ENGINE NOT Started!",
                        "The selected chess could not be started. Please contact an administrator.",
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

        // From here on out get the chess engine options and let the user select
        System.err.println("Chess engine started!");

        //GridPane chessBoardGUI = new UIChessBoard(true);
        contentAnchor.getChildren().add(new UIChessBoard(true, chessEngine, ircClient));
    }

    @FXML
    private void stopChessGame( javafx.event.ActionEvent event ){

        try{
            chessEngine.stopChessEngine(ircClient);
            System.out.println("Amount of children inside contentAnchor");
            System.out.println(contentAnchor.getChildren().size());
            contentAnchor.getChildren().remove(0);
        }catch (IOException e) {
            PopUpMessages.showAlert("IOException", "SENDING OR RECEIVING ERROR",
                    "There was an error with sending or receiving using the client. " +
                            "Possibly lost connection to the server",
                    Alert.AlertType.ERROR);
            return;
        } catch (ProtocolException e) {
            if(e.getMessage().equals(ProtocolErrors.CHESSENGINE_NOT_FOUND.toString())){
                PopUpMessages.showAlert("ProtocolError", "CHESS ENGINE NOT FOUND!",
                        "The selected chess engine could not be found.",
                        Alert.AlertType.ERROR);
                return;
            }
            else if(e.getMessage().equals(ProtocolErrors.CHESSENGINE_NOT_STARTED.toString())){
                PopUpMessages.showAlert("ProtocolError", "CHESS ENGINE NOT Started!",
                        "The selected chess could not be started. Please contact an administrator.",
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
    }

    // /////////////////// //
    // Instance variables. //
    // /////////////////// //

    @FXML
    private AnchorPane contentAnchor;

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
        chessEngine = mainApp.getChessEngine();
    }

}
