/*
 *@(#) ibo1.ChessSpecific.ChessPiece.java 0.1 2023/03/17
 *
 */
package ibo1.ChessSpecific;

/**
 * ChessPiece - Gives basic Structure to all chess pieces
 * <p>
 * 3rd party code is used in this class. It is an adaptions from GitHub user: Stevoisiak.
 * Link to the original code: <a href="https://github.com/Stevoisiak/JavaFX-Online-Chess">Github Link</a>.
 * This class represents the ChessPiece.java file
 * Accessed 30th of April 2023
 *
 *
 * @author Omar Ibrahim
 * @version 0.1 ( Initial development ).
 * @version 0.2 ( Second development ).
 * @version 1.0 ( Initial release ).
 *
 * @see Cell
 * @see ChessBoard
 * @see Bishop
 * @see King
 * @see Knight
 * @see Pawn
 * @see Queen
 * @see Rook
 */
public abstract class ChessPiece {
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
     * Protected field if the color of the chess piece is white.
     */
    protected boolean isWhite;

    // ///////////// //
    // Constructors. //
    // ///////////// //

    /**
     * Constructor for creating a ChessPiece with a color.
     *
     * @param isWhite is color of the piece white
     */
    public ChessPiece(boolean isWhite){
        this.isWhite = isWhite;
    }

    // ////////////////////// //
    // Read/Write properties. //
    // ////////////////////// //

    // ///////////////////// //
    // Read-only properties. //
    // ///////////////////// //


    /**
     * @return the color of the chess piece as a string
     */
    public String getColor() {
        if(this.isWhite) return "white";
        else return "black";
    }


    // //////// //
    // Methods. //
    // //////// //

    /**
     * @return the name of the piece
     */
    public abstract String getName();
}
