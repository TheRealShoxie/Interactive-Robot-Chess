/*
 *@(#) ChessSpecific.Cell.java 0.1 2023/03/17
 *
 * Copyright (c) Omar Ibrahim
 * All rights reserved.
 */
package ChessSpecific;


/**
 * Cell - Interactive Robot Chess Cell class is the internal representation of a chess cell.
 * It can hold chess pieces, release and set them.
 * Holds its own position in the chessboard and if the cell is a light cell or not.
 *
 * <p>
 * 3rd party code is used in this class. It is an adaptions from GitHub user: Stevoisiak.
 * Link to the original code: <a href="https://github.com/Stevoisiak/JavaFX-Online-Chess">Github Link</a>
 * This class represents the Space.java file
 *
 *
 * @author Omar Ibrahim
 * @version 0.1 ( Initial development ).
 * @version 0.2 ( Second development ).
 * @version 1.0 ( Initial release ).
 *
 * @see ChessBoard
 * @see ChessPiece
 * @see Bishop
 * @see King
 * @see Knight
 * @see Pawn
 * @see Queen
 * @see Rook
 */
public class Cell {
    // ////////// //
    // Constants. //
    // ////////// //

    // //////////////// //
    // Class variables. //
    // //////////////// //

    // ////////////// //
    // Class methods. //
    // ////////////// //


    // /////////////////// //
    // Instance variables. //
    // /////////////////// //

    /**
     * Variable to hold a chess piece of type ChessPiece
     */
    private ChessPiece chessPiece;

    /**
     * Variable to hold the x position of the cell in the chess board
     */
    private final int xPos;

    /**
     * Variable to hold the y position of the cell in the chess board
     */
    private final int yPos;

    /**
     * Variable to tell if the cell is a light cell or not
     */
    private final boolean lightCell;


    // ///////////// //
    // Constructors. //
    // ///////////// //

    /**
     * Constructor for a cell
     *
     * @param light is the cell a light cell
     * @param x X position in the chessboard
     * @param y Y position in the chessboard
     */
    public Cell(boolean light, int x, int y){
        this.xPos = x;
        this.yPos = y;
        this.chessPiece = null;
        this.lightCell = light;
    }

    // ////////////////////// //
    // Read/Write properties. //
    // ////////////////////// //


    /**
     * Method to set a chess Piece
     *
     * @param chessPiece chess piece to be set
     */
    public void setChessPiece(ChessPiece chessPiece) {
        this.chessPiece = chessPiece;
    }


    /**
     * Method to get the chess piece
     *
     * @return the chess piece that the cell holds
     */
    public ChessPiece getChessPiece() {
        return chessPiece;
    }


    /**
     * @return the x position of the cell in the chessboard
     */
    public int getXPos() {
        return xPos;
    }


    /**
     * @return the y position of the cell in the chessboard
     */
    public int getYPos() {
        return yPos;
    }


    /**
     * @return if the cell is a light cell
     */
    public boolean isLightCell() {
        return lightCell;
    }

    // ///////////////////// //
    // Read-only properties. //
    // ///////////////////// //

    // //////// //
    // Methods. //
    // //////// //

    /**
     * Method to release a chess piece from the cell and return it
     *
     * @return the release chess piece
     */
    public ChessPiece releaseChessPiece(){
        ChessPiece tempChessPiece = this.chessPiece;
        setChessPiece(null);
        return tempChessPiece;
    }
}
