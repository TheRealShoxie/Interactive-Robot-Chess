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
import javafx.scene.layout.GridPane;

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
        Cell[][] cells = chessBoard.getCells();

        for(int x = 0; x < cells[0].length; x++){
            for(int y = 0; y < cells[1].length; y++) {
                spaces[x][y].setCell(cells[x][y]);
            }
        }
    }

    /**
     * ------------------------------------------------------------------------------------

     */

    // /////////////////// //
    // Instance variables. //
    // /////////////////// //
    Space[][] spaces = new Space[8][8];
    ChessBoard chessBoard;

    // ///////////// //
    // Constructors. //
    // ///////////// //

    public UIChessBoard(boolean playerIsWhite, ChessEngine chessEngine, IRCClient ircClient){
        super();

        chessBoard = new ChessBoard(playerIsWhite, chessEngine, ircClient);

        for(int x = 0; x < spaces[0].length; x++){
            for(int y = 0; y < spaces[1].length; y++) {


                Space space = new Space(this.chessBoard.getCellAtPosition(x,y));
                spaces[x][y] = space;

                this.add(spaces[x][y],x,y);

                final int xVal = x;
                final int yVal = y;

                spaces[x][y].setOnAction(e -> {
                    this.chessBoard.onCellClick(xVal, yVal);
                    this.updateBoardState();
                });

            }
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

}
