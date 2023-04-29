/*
 *@(#) Utility.DataChecker.java 0.1 2023/03/17
 *
 * Copyright (c) Omar Ibrahim
 * All rights reserved.
 */
package ibo1.UI;

import ChessSpecific.*;
import Client.IRCClient;
import Protocol.ChessEngine;
import ibo1.UIElements.Space;
import ibo1.Utility.PopUpMessages;
import javafx.collections.FXCollections;
import javafx.collections.ObservableList;
import javafx.scene.layout.GridPane;

import java.util.ArrayList;
import java.util.List;

/**
 * ClassName - ClassDescription initial
 * <p>
 * What it does
 * @author Omar Ibrahim
 * @version 0.1 ( Initial development ).
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

    private void updateBoardState(){
        int position = 0;
        for(int x = 0; x < 8; x++){
            for(int y = 0; y < 8; y++) {
                position = y +(x*8);
                observableListSpaces.get(position).setCell(chessBoard.getCellAtPosition(x,y));
            }
        }
    }


    private void setActiveSpace(Space space){
        if(this.activeSpace != null) this.activeSpace.getStyleClass().removeAll("chess-space-active");

        this.activeSpace = space;

        if(this.activeSpace != null) this.activeSpace.getStyleClass().add("chess-space-active");
    }

    private void chessEngineMove(){

        this.chessBoard.chessEngineMove();

        updateBoardState();
        this.setDisable(false);
    }

    /**
     * ------------------------------------------------------------------------------------

     */

    // /////////////////// //
    // Instance variables. //
    // /////////////////// //
    ObservableList<Space> observableListSpaces = FXCollections.observableArrayList();
    ChessBoard chessBoard;
    Space activeSpace;

    // ///////////// //
    // Constructors. //
    // ///////////// //

    public UIChessBoard(boolean playerIsWhite, ChessEngine chessEngine, IRCClient ircClient){
        super();

        chessBoard = new ChessBoard(playerIsWhite, chessEngine, ircClient);

        for(int x = 0; x < 8; x++){
            for(int y = 0; y < 8; y++) {

                //Creating the space
                Space space = new Space(this.chessBoard.getCellAtPosition(x,y));
                // Adding the space to the observable list
                observableListSpaces.add(space);

                int pos = y +(x*8);
                // Adding the observableListSpace at x and y
                this.add(observableListSpaces.get(pos),x,y);

                final int xVal = x;
                final int yVal = y;

                observableListSpaces.get(pos).setOnAction(e -> {
                    this.onSpaceClick(xVal, yVal);
                });
            }
        }

        if(!playerIsWhite){
            this.setDisable(true);
            this.chessEngineMove();
        }

    }

    // ////////////////////// //
    // Read/Write properties. //
    // ////////////////////// //

    // ///////////////////// //
    // Read-only properties. //
    // ///////////////////// //

    // //////// //
    // Methods. //
    // //////// //
    public void onSpaceClick(int x, int y){
        int position = y + (x*8);

        Space clickedSpace = observableListSpaces.get(position);

        if(activeSpace != null
                && activeSpace.getCell().getChessPiece() != null
                && activeSpace.getCell().getChessPiece() != clickedSpace.getCell().getChessPiece()){

            int activeCellX = activeSpace.getCell().getXPos();
            int activeCellY = activeSpace.getCell().getYPos();
            int clickedCellX = clickedSpace.getCell().getXPos();
            int clickedCellY = clickedSpace.getCell().getYPos();

            boolean madeMove;

            if(activeSpace.getCell().getChessPiece().getName().toLowerCase().charAt(0) == 'p'
                && (activeSpace.getCell().getYPos() == 1 || activeSpace.getCell().getYPos() == 6)
                && (clickedSpace.getCell().getYPos() == 0 || clickedSpace.getCell().getYPos() == 7)) {

                List<String> promotion = new ArrayList<>();
                promotion.add("Queen");
                promotion.add("Knight");
                promotion.add("Rook");
                promotion.add("Bishop");

                String selectedPromotion = PopUpMessages.showChoiceDialogPromotion(promotion);

                char toPromoteInto;

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

                madeMove = this.chessBoard.playerMove(activeCellX, activeCellY,
                        clickedCellX, clickedCellY, toPromoteInto);
            }
            else{
                madeMove = this.chessBoard.playerMove(activeCellX, activeCellY,
                        clickedCellX, clickedCellY, ' ');
            }




            if(!madeMove) return;

            updateBoardState();

            this.setActiveSpace(null);
            this.setDisable(true);

            this.chessEngineMove();
        }
        else if(activeSpace != null && activeSpace.getCell().getChessPiece() == clickedSpace.getCell().getChessPiece()){
            this.setActiveSpace(null);
        }
        else{
            if(observableListSpaces.get(position).getCell().getChessPiece() != null){
                this.setActiveSpace(observableListSpaces.get(position));
            }
        }

    }

}
