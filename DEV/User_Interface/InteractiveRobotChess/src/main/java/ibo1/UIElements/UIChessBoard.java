/*
 *@(#) UI.UIChessBoard.java 0.1 2023/03/17
 *
 * Copyright (c) Omar Ibrahim
 * All rights reserved.
 */
package ibo1.UIElements;

import ChessSpecific.*;
import Client.IRCClient;
import CustomException.ProtocolException;
import Protocol.ChessEngine;
import ibo1.Utility.PopUpMessages;
import javafx.collections.FXCollections;
import javafx.collections.ObservableList;
import javafx.scene.control.Alert;
import javafx.scene.control.Button;
import javafx.scene.image.ImageView;
import javafx.scene.layout.GridPane;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

/**
 * UIChessBoard - Interactive Robot Chess UIChessBoard class is the UI representation of a chess board
 * <p>
 * This class extends the GridPane element from JavaFX.
 * To display the chess board graphically.
 *
 *
 * <p>
 * 3rd party code is used in this class. It is an adaptions from GitHub user: Stevoisiak.
 * Link to the original code: <a href="https://github.com/Stevoisiak/JavaFX-Online-Chess">Github Link</a>
 * This class represents the ChessBoard.java file
 *
 *
 *
 * @author Omar Ibrahim
 * @version 0.1 ( Initial development ).
 * @version 0.2 ( Second development ).
 * @version 1.0 ( Initial release ).
 *
 * @see ImageView
 * @see Button
 * @see Cell
 * @see ChessPiece
 * @see ChessBoard
 * @see Space
 * @see GridPane
 * @see IRCClient
 * @see ChessEngine
 */
public class UIChessBoard extends GridPane{
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
     * Updates the chess board state to represent the IRC_API chessboard
     */
    private void updateBoardState(){
        int position = 0;

        // Cycle through each cell of the IRC_API chessboard
        for(int x = 0; x < 8; x++){
            for(int y = 0; y < 8; y++) {
                position = y +(x*8);
                observableListSpaces.get(position).setCell(chessBoard.getCellAtPosition(x,y));
            }
        }
    }


    /**
     * Sets the active cell and changes the color of it
     *
     * @param space The space to set as active
     */
    private void setActiveSpace(Space space){

        // If current active space is not null then deselect it graphically
        if(this.activeSpace != null) this.activeSpace.getStyleClass().removeAll("chess-space-active");

        // Set the active space to the supplied space
        this.activeSpace = space;

        // If the active space is not empty then select it graphically
        if(this.activeSpace != null) this.activeSpace.getStyleClass().add("chess-space-active");
    }

    /**
     * Make a chess engine move on by calling the chessboard chessengine move
     *
     * @throws ProtocolException thrown by chessBoard.chessEngineMove()
     * @throws IOException thrown by chessBoard.chessEngineMove()
     */
    private void chessEngineMove() throws ProtocolException, IOException {

        // Make a chessEngineMove
        this.chessBoard.chessEngineMove();

        // Update the board state
        updateBoardState();

        // Enable the graphically interface to be interactable again
        this.setDisable(false);
    }

    /**
     * ------------------------------------------------------------------------------------

     */

    // /////////////////// //
    // Instance variables. //
    // /////////////////// //

    /**
     * Observable list representation of a chess board for the UI
     */
    ObservableList<Space> observableListSpaces = FXCollections.observableArrayList();

    /**
     * The reference to the IRC_API chess board
     */
    ChessBoard chessBoard;

    /**
     * The current active space
     */
    Space activeSpace;

    // ///////////// //
    // Constructors. //
    // ///////////// //

    /**
     * Constructor for the UI Chess board
     *
     * @param playerIsWhite if the player is white
     * @param chessEngine supply a chess-engine for the chessboard
     * @param ircClient supply an ircClient for the chessboard
     */
    public UIChessBoard(boolean playerIsWhite, ChessEngine chessEngine, IRCClient ircClient){

        // Initialize the super from gridpane
        super();

        // Create the IRC_API Chessboard
        chessBoard = new ChessBoard(playerIsWhite, chessEngine, ircClient);

        // Iterate through each space for the UI chess board
        for(int x = 0; x < 8; x++){
            for(int y = 0; y < 8; y++) {

                //Creating the space
                Space space = new Space(this.chessBoard.getCellAtPosition(x,y));
                // Adding the space to the observable list
                observableListSpaces.add(space);

                int pos = y +(x*8);
                // Adding the observableListSpace at x and y
                this.add(observableListSpaces.get(pos),x,y);

                // Setting final x and y values for the space
                final int xVal = x;
                final int yVal = y;

                // For each space set an action
                observableListSpaces.get(pos).setOnAction(e -> {

                    // try on space click
                    try {
                        this.onSpaceClick(xVal, yVal);
                    }
                    // Catching IOException
                    catch (ProtocolException ex) {
                        PopUpMessages.showAlert("Move Error", ex.getMessage().toString(),
                                "Please refer to the above stated error for information on what went wrong.",
                                Alert.AlertType.ERROR);

                    }
                    // Catching Protocol exception
                    catch (IOException ex) {
                        PopUpMessages.showAlert("IOException", "SENDING OR RECEIVING ERROR",
                                "There was an error with sending or receiving using the client. " +
                                        "Possibly lost connection to the server",
                                Alert.AlertType.ERROR);
                    }
                });
            }
        }

        // If the player is not white disable the graphical chessboard and make a chessEngine move
        if(!playerIsWhite){
            this.setDisable(true);
            // Trying to make a chessEngine move
            try{
                this.chessEngineMove();
            }
            // Catching IOException
            catch (ProtocolException ex) {
                PopUpMessages.showAlert("Move Error", ex.getMessage().toString(),
                        "Please refer to the above stated error for information on what went wrong.",
                        Alert.AlertType.ERROR);

            }
            // Catching Protocol exception
            catch (IOException ex) {
                PopUpMessages.showAlert("IOException", "SENDING OR RECEIVING ERROR",
                        "There was an error with sending or receiving using the client. " +
                                "Possibly lost connection to the server",
                        Alert.AlertType.ERROR);
            }
        }

    }

    // ////////////////////// //
    // Read/Write properties. //
    // ////////////////////// //

    // ///////////////////// //
    // Read-only properties. //
    // ///////////////////// //

    /**
     * Called when a UI space in the UI Chess board is clicked.
     * It makes a player move to the system if an active space was already set.
     * Otherwise, it sets an active space
     *
     * @param x the X position of the space
     * @param y the Y position of the space
     * @throws ProtocolException thrown by chessBoard.playerMove()
     * @throws IOException thrown by chessBoard.playerMove()
     */
    // //////// //
    // Methods. //
    // //////// //
    public void onSpaceClick(int x, int y) throws ProtocolException, IOException {

        // Getting the position from the observable list of spaces
        int position = y + (x*8);

        // Getting the clicked space
        Space clickedSpace = observableListSpaces.get(position);

        // Checking if active space is not null, and it has a chess piece and active space is not clicked space
        if(activeSpace != null
                && activeSpace.getCell().getChessPiece() != null
                && activeSpace.getCell().getChessPiece() != clickedSpace.getCell().getChessPiece()){

            // Getting the activeSpace and clickedSpace Position
            int activeSpaceX = activeSpace.getCell().getXPos();
            int activeSpaceY = activeSpace.getCell().getYPos();
            int clickedSpaceX = clickedSpace.getCell().getXPos();
            int clickedSpaceY = clickedSpace.getCell().getYPos();

            // Boolean for checking if move was made
            boolean madeMove;

            // Checking if the active piece is a pawn, and its position is at the end of the board
            if(activeSpace.getCell().getChessPiece().getName().toLowerCase().charAt(0) == 'p'
                && (activeSpace.getCell().getYPos() == 1 || activeSpace.getCell().getYPos() == 6)
                && (clickedSpace.getCell().getYPos() == 0 || clickedSpace.getCell().getYPos() == 7)) {

                // Then we have a promotion move
                // Create a list of possible promotions
                List<String> promotion = new ArrayList<>();
                promotion.add("Queen");
                promotion.add("Knight");
                promotion.add("Rook");
                promotion.add("Bishop");

                // Show a popup and save the selected piece
                String selectedPromotion = PopUpMessages.showChoiceDialogPromotion(promotion);

                char toPromoteInto;

                // Checking if the selected piece matches one of these if yes then set it as the to promote to
                switch (selectedPromotion){
                    case "Queen":
                        toPromoteInto = 'q';
                        break;
                    case "Rook":
                        toPromoteInto = 'r';
                        break;
                    case "Knight":
                        toPromoteInto = 'n';
                        break;
                    case "Bishop":
                        toPromoteInto = 'b';
                        break;
                    default:
                        toPromoteInto = 'a';
                        break;
                }

                // Make a place move with supplied information
                madeMove = this.chessBoard.playerMove(activeSpaceX, activeSpaceY,
                        clickedSpaceX, clickedSpaceY, toPromoteInto);
            }
            else{

                // We have a normal chess move thus make a normal move
                madeMove = this.chessBoard.playerMove(activeSpaceX, activeSpaceY,
                        clickedSpaceX, clickedSpaceY, ' ');
            }


            // Checking if move was not successfull then return
            if(!madeMove) return;


            // UpdateBoardState
            updateBoardState();

            // Set active space to null and disable the chessboard gui
            this.setActiveSpace(null);
            this.setDisable(true);

            // Make a chess engine move
            this.chessEngineMove();
        }

        // Otherwise check if active space not null, and the chess piece is the same as the clicked one
        else if(activeSpace != null && activeSpace.getCell().getChessPiece() == clickedSpace.getCell().getChessPiece()){

            // Then deselect that space
            this.setActiveSpace(null);
        }

        // Otherwise no activeSpace set
        else{

            // Check if the space clicked has a chess piece
            if(observableListSpaces.get(position).getCell().getChessPiece() != null){

                //Set this space as the new active space
                this.setActiveSpace(observableListSpaces.get(position));
            }
        }

    }

}
