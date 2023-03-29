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
        //white pieces
        this.cells[0][7].setChessPiece(new Rook(true));
        this.cells[1][7].setChessPiece(new Knight(true));
        this.cells[2][7].setChessPiece(new Bishop(true));
        this.cells[3][7].setChessPiece(new Queen(true));
        this.cells[4][7].setChessPiece(new King(true));
        this.cells[5][7].setChessPiece(new Bishop(true));
        this.cells[6][7].setChessPiece(new Knight(true));
        this.cells[7][7].setChessPiece(new Rook(true));

        for(int i = 0; i < this.cells[0].length; i++)
            this.cells[i][6].setChessPiece(new Pawn(true));

        //black pieces
        this.cells[0][0].setChessPiece(new Rook(false));
        this.cells[1][0].setChessPiece(new Knight(false));
        this.cells[2][0].setChessPiece(new Bishop(false));
        this.cells[3][0].setChessPiece(new Queen(false));
        this.cells[4][0].setChessPiece(new King(false));
        this.cells[5][0].setChessPiece(new Bishop(false));
        this.cells[6][0].setChessPiece(new Knight(false));
        this.cells[7][0].setChessPiece(new Rook(false));

        for(int i = 0; i < this.cells[0].length; i++)
            this.cells[i][1].setChessPiece(new Pawn(false));
    }

    private boolean pawnValidityCheck(MoveInfo moveInfo){
        Cell oldCell = moveInfo.getOldCell();
        Cell newCell = moveInfo.getNewCell();

        ChessPiece chessPiece = oldCell.getChessPiece();

        // If we are not dealing with a pawn return true
        if(!chessPiece.getName().equals("pawn")) return true;

        if(moveInfo.getGapX() == 0){
            int colorMod = moveInfo.getGapY() / Math.abs(moveInfo.getGapY());

            for(int c = 1; c <= Math.abs(moveInfo.getGapY()); c++){
                if(cells[oldCell.getXPos()][oldCell.getXPos() + (c*colorMod)].isOccupied()) return false;
            }
        }
        else{
            if((!newCell.isOccupied()) ||
                    (chessPiece.getColor() == newCell.getChessPiece().getColor())
            ) return false;
        }

        return true;
    }

    private boolean moveIsValid(MoveInfo moveInfo){
        Cell oldCell;
        Cell newCell;
        ChessPiece chessPiece;
        MoveList[] possibleMoves;

        // If we have an empty move return false
        if(moveInfo == null) return false;

        // Trying to extract oldCell and newCell from moveInfo
        try{
            oldCell = moveInfo.getOldCell();
            newCell = moveInfo.getNewCell();
        } catch(NullPointerException e) {return false;}

        // Checking if the cell to move from is empty.
        if(!oldCell.isOccupied()) return false;

        chessPiece = oldCell.getChessPiece();
        possibleMoves = chessPiece.getChessPieceMove();


        int multiMoveCount;
        int stretchedMoveX;
        int stretchedMoveY;

        for(MoveList move : possibleMoves){

            //Piece only has 1 cell it can move
            multiMoveCount = 1;

            //Checking if chessPiece can move multiple cells
            if(!chessPiece.usesSingleMove()) multiMoveCount = 8;

            // Setting if has collided to false
            boolean hasCollided = false;
            for(int moveCount = 1; moveCount <= multiMoveCount; moveCount++){
                //If we have collided we want to stop searching for chessPieces that can move multiple cells
                if(hasCollided) break;

                stretchedMoveX = move.getX() * moveCount;
                stretchedMoveY = move.getY() * moveCount;

                Cell toCheckCell;

                try{
                    toCheckCell = this.cells[oldCell.getXPos()+stretchedMoveX][oldCell.getYPos() + stretchedMoveY];
                }catch(Exception e){break;}

                // Checking if cell is occupied
                if(toCheckCell.isOccupied()){
                    hasCollided = true;
                    boolean piecesSameColor = toCheckCell.getChessPiece().getColor()
                                              == oldCell.getChessPiece().getColor();

                    if(piecesSameColor) break;

                    if(toCheckCell.getChessPiece() != newCell.getChessPiece()) break;
                }

                if(moveInfo.getGapX() == stretchedMoveX && moveInfo.getGapY() == stretchedMoveY){

                    if(pawnValidityCheck(moveInfo)) return true;


                    chessPiece.setHasMoved(true);
                    break;
                }
            }
        }

        return false;
    }

    private boolean processMove(MoveInfo moveInfo){
        if(moveIsValid(moveInfo)){
            Cell oldCell = moveInfo.getOldCell();
            Cell newCell = moveInfo.getNewCell();

            newCell.setChessPiece(oldCell.releaseChessPiece());
            return true;
        }
        else{
            return false;
        }
    }

    // /////////////////// //
    // Instance variables. //
    // /////////////////// //
    Cell[][] cells;
    Cell activeCell;

    // ///////////// //
    // Constructors. //
    // ///////////// //
    public ChessBoard(boolean playerIsWhite){

        //Setting up cell grid for the board
        this.cells = new Cell[8][8];


        for(int x = 0; x < cells[0].length; x++){
            for(int y = 0; y < cells[1].length; y++){
                Cell cell;

                //Checking if cell should be a light-colored cell or dark-colored
                if( (x+y)%2 != 0) cell = new Cell(true, x, y);
                else cell = new Cell(false, x, y);

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

            MoveInfo moveInfo;
            moveInfo = new MoveInfo(activeCell, clickedCell);

            //Process move
            if(processMove(moveInfo)){
                System.err.println("Need to send move command");
            }

            //Send move

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
