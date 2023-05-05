/*
 *@(#) ibo1.ChessSpecific.Queen.java 0.1 2023/03/17
 *
 */
package ibo1.ChessSpecific;

/**
 * Queen - Interactive Robot Chess Queen Class that represents a queen.
 *
 * Extends the ChessPiece class.
 *
 * @author Omar Ibrahim
 * @version 0.1 ( Initial development ).
 * @version 0.2 ( Second development ).
 * @version 1.0 ( Initial release ).
 *
 * @see ChessPiece
 * @see Cell
 * @see ChessBoard
 */
public class Queen extends ChessPiece {
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

    // ///////////// //
    // Constructors. //
    // ///////////// //

    /**
     * Constructor for creating a Queen with a color.
     *
     * @param isWhite is color of the piece white
     */
    public Queen(boolean isWhite){
        super(isWhite);
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

    /**
     * @return the name of the piece
     */
    public String getName(){return "queen";}

    /**
     * @return a single String value for the piece
     */
    public String toString(){
        if(isWhite) return "Q";
        else return "q";
    }
}
