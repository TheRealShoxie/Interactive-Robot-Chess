/*
 *@(#) Utility.DataChecker.java 0.1 2023/03/17
 *
 * Copyright (c) Omar Ibrahim
 * All rights reserved.
 */
package ChessSpecific;

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

        if(this.playerIsWhite){
            whiteY = 7;
            whitePawnY = 6;
            blackY = 0;
            blackPawnY = 1;
        }
        else{
            whiteY = 0;
            whitePawnY = 1;
            blackY = 7;
            blackPawnY = 6;
        }

        //white pieces
        this.cells[0][whiteY].setChessPiece(new Rook(true));
        this.cells[1][whiteY].setChessPiece(new Knight(true));
        this.cells[2][whiteY].setChessPiece(new Bishop(true));
        this.cells[3][whiteY].setChessPiece(new Queen(true));
        this.cells[4][whiteY].setChessPiece(new King(true));
        this.cells[5][whiteY].setChessPiece(new Bishop(true));
        this.cells[6][whiteY].setChessPiece(new Knight(true));
        this.cells[7][whiteY].setChessPiece(new Rook(true));

        for(int i = 0; i < this.cells[0].length; i++)
            this.cells[i][whitePawnY].setChessPiece(new Pawn(true));

        //black pieces
        this.cells[0][blackY].setChessPiece(new Rook(false));
        this.cells[1][blackY].setChessPiece(new Knight(false));
        this.cells[2][blackY].setChessPiece(new Bishop(false));
        this.cells[3][blackY].setChessPiece(new Queen(false));
        this.cells[4][blackY].setChessPiece(new King(false));
        this.cells[5][blackY].setChessPiece(new Bishop(false));
        this.cells[6][blackY].setChessPiece(new Knight(false));
        this.cells[7][blackY].setChessPiece(new Rook(false));

        for(int i = 0; i < this.cells[0].length; i++)
            this.cells[i][blackPawnY].setChessPiece(new Pawn(false));
    }

    private boolean processMove(Cell oldCell, Cell newCell){
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
        System.err.println(String.valueOf(moveCmd));

        System.err.println("Send move to chessEngine");
        if(false){
            System.err.println();
        }

        return true;
    }

    // /////////////////// //
    // Instance variables. //
    // /////////////////// //
    Cell[][] cells;
    Cell activeCell;
    boolean playerIsWhite;

    // ///////////// //
    // Constructors. //
    // ///////////// //
    public ChessBoard(boolean playerIsWhite){

        //Setting up cell grid for the board
        this.cells = new Cell[8][8];
        this.playerIsWhite = playerIsWhite;


        for(int x = 0; x < cells[0].length; x++){
            for(int y = 0; y < cells[1].length; y++){
                Cell cell;

                //Checking if cell should be a light-colored cell or dark-colored
                if( (x+y)%2 != 0) cell = new Cell(false, x, y);
                else cell = new Cell(true, x, y);

                if(playerIsWhite) cells[x][y] = cell;
                else cells[x][y] = cell;
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

    public void onCellClick(int x, int y){
        Cell clickedCell = cells[x][y];

        //Checking if activeCell not empty and activeCell piece not null and activeCell not clickedCell
        if(activeCell != null
                && activeCell.getChessPiece() != null
                && activeCell.getChessPiece() != clickedCell.getChessPiece()){

            if(processMove(activeCell, clickedCell)){
                System.err.println("Need to get move by chessEngine");
            } else{
                System.err.println("Move was illegal");
            }

            // Reset activeCell
            this.activeCell = null;

        }
        //Otherwise
        else{
            //Check if clicked cell piece not null then set active cell as clicked cell
            if(clickedCell.getChessPiece() != null) this.activeCell = clickedCell;
        }
    }
}
