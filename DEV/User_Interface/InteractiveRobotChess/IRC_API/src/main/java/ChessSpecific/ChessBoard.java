/*
 *@(#) Utility.DataChecker.java 0.1 2023/03/17
 *
 * Copyright (c) Omar Ibrahim
 * All rights reserved.
 */
package ChessSpecific;

import Client.IRCClient;
import Protocol.ChessEngine;

import java.nio.charset.Charset;

/**
 * ClassName - ClassDescription initial
 * <p>
 * What it does
 * @author Omar Ibrahim
 * @version 0.1 ( Initial development ).
 */
public class ChessBoard {
    // ////////// //
    // Constants. //
    // ////////// //

    // //////////////// //
    // Class variables. //
    // //////////////// //

    // ////////////// //
    // Class methods. //
    // ////////////// //

    private void defineStartPositions(){

        int whiteY;
        int whitePawnY;
        int blackY;
        int blackPawnY;
        int kingPos;
        int queenPos;

        if(this.playerIsWhite){
            whiteY = 7;
            whitePawnY = 6;
            blackY = 0;
            blackPawnY = 1;
            kingPos = 4;
            queenPos = 3;
        }
        else{
            whiteY = 0;
            whitePawnY = 1;
            blackY = 7;
            blackPawnY = 6;
            kingPos = 3;
            queenPos = 4;
        }

        //white pieces
        this.cells[0][whiteY].setChessPiece(new Rook(true));
        this.cells[1][whiteY].setChessPiece(new Knight(true));
        this.cells[2][whiteY].setChessPiece(new Bishop(true));
        this.cells[queenPos][whiteY].setChessPiece(new Queen(true));
        this.cells[kingPos][whiteY].setChessPiece(new King(true));
        this.cells[5][whiteY].setChessPiece(new Bishop(true));
        this.cells[6][whiteY].setChessPiece(new Knight(true));
        this.cells[7][whiteY].setChessPiece(new Rook(true));

        for(int i = 0; i < this.cells[0].length; i++)
            this.cells[i][whitePawnY].setChessPiece(new Pawn(true));

        //black pieces
        this.cells[0][blackY].setChessPiece(new Rook(false));
        this.cells[1][blackY].setChessPiece(new Knight(false));
        this.cells[2][blackY].setChessPiece(new Bishop(false));
        this.cells[queenPos][blackY].setChessPiece(new Queen(false));
        this.cells[kingPos][blackY].setChessPiece(new King(false));
        this.cells[5][blackY].setChessPiece(new Bishop(false));
        this.cells[6][blackY].setChessPiece(new Knight(false));
        this.cells[7][blackY].setChessPiece(new Rook(false));

        for(int i = 0; i < this.cells[0].length; i++)
            this.cells[i][blackPawnY].setChessPiece(new Pawn(false));
    }

    private void convertToMoveFormat(Cell oldCell, Cell newCell, StringBuilder sb){
        char[] moveCmd = new char[4];
        if(this.playerIsWhite){
            moveCmd[0] = (char)(oldCell.getXPos()+97);
            moveCmd[1] = Character.forDigit(7-oldCell.getYPos()+1, 10);
            moveCmd[2] = (char)(newCell.getXPos()+97);
            moveCmd[3] = Character.forDigit(7-newCell.getYPos()+1, 10);
        }else{
            moveCmd[0] = (char)(7-oldCell.getXPos()+97);
            moveCmd[1] = Character.forDigit(oldCell.getYPos()+1, 10);
            moveCmd[2] = (char)(7-newCell.getXPos()+97);
            moveCmd[3] = Character.forDigit(newCell.getYPos()+1, 10);
        }
        sb.append(String.valueOf(moveCmd));
    }

    private int[] convertToMoveFormat(String move){
        char[] moveCmd = move.toLowerCase().toCharArray();

        int oldCellX = 0;
        int oldCellY = 0;
        int newCellX = 0;
        int newCellY = 0;

        if(this.playerIsWhite){
            oldCellX = (int)moveCmd[0] - 97;
            oldCellY = 8 - Character.getNumericValue(moveCmd[1]);
            newCellX = (int)moveCmd[2] - 97;
            newCellY = 8 - Character.getNumericValue(moveCmd[3]);

        }else{
            oldCellX = 7 - ((int)moveCmd[0] - 97);
            oldCellY = Character.getNumericValue(moveCmd[1]) - 1;
            newCellX = 7 - ((int)moveCmd[2] - 97);
            newCellY = Character.getNumericValue(moveCmd[3]) - 1;
        }

        int returnedInt[] = new int[4];
        returnedInt[0] = oldCellX;
        returnedInt[1] = oldCellY;
        returnedInt[2] = newCellX;
        returnedInt[3] = newCellY;

        return returnedInt;

    }

    // /////////////////// //
    // Instance variables. //
    // /////////////////// //
    Cell[][] cells;
    boolean playerIsWhite;
    ChessEngine chessEngine;
    IRCClient ircClient;

    // ///////////// //
    // Constructors. //
    // ///////////// //
    public ChessBoard(boolean playerIsWhite, ChessEngine chessEngine, IRCClient ircClient){

        //Setting up cell grid for the board
        this.cells = new Cell[8][8];
        this.playerIsWhite = playerIsWhite;
        this.chessEngine = chessEngine;
        this.ircClient = ircClient;


        for(int x = 0; x < cells[0].length; x++){
            for(int y = 0; y < cells[1].length; y++){
                Cell cell;

                //Checking if cell should be a light-colored cell or dark-colored
                if( (x+y)%2 != 0) cell = new Cell(false, x, y);
                else cell = new Cell(true, x, y);

                cells[x][y] = cell;
            }
        }

        this.defineStartPositions();
    }

    // ////////////////////// //
    // Read/Write properties. //
    // ////////////////////// //

    // ///////////////////// //
    // Read-only properties. //
    // ///////////////////// //

    public Cell[][] getCells() {
        return cells;
    }
    // //////// //
    // Methods. //
    // //////// //

    public Cell getCellAtPosition(int x, int y){
        return this.cells[x][y];
    }

    public boolean playerMove(int activeCellX, int activeCellY, int clickedCellX, int clickedCellY){

        StringBuilder move = new StringBuilder();
        convertToMoveFormat(cells[activeCellX][activeCellY], cells[clickedCellX][clickedCellY], move);

        System.out.println("__________________________________");
        System.out.println("Sending move command: " +move);
        System.out.println("__________________________________");

        try{
            chessEngine.makePlayerMove(ircClient, move.toString());
        } catch(Exception e){
            System.err.println(e.getMessage());
            return false;
        }

        ChessPiece tempChessPiece = cells[activeCellX][activeCellY].releaseChessPiece();
        cells[clickedCellX][clickedCellY].setChessPiece(tempChessPiece );

        return true;
    }

    public int[] chessEngineMove(){

        String chessEngineMoved = "";

        try{
            chessEngineMoved = chessEngine.makeChessEngineMove(ircClient);
        } catch(Exception e){
            System.err.println(e.getMessage());
            return new int[0];
        }

        System.out.println("__________________________________");
        System.out.println("Received move command: " +chessEngineMoved);
        System.out.println("__________________________________");

        int cellPos[] = convertToMoveFormat(chessEngineMoved);

        ChessPiece tempChessPiece = cells[cellPos[0]][cellPos[1]].releaseChessPiece();
        cells[cellPos[2]][cellPos[3]].setChessPiece(tempChessPiece );

        return cellPos;

    }

}
