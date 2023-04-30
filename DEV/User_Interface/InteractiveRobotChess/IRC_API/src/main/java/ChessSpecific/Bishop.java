/*
 *@(#) ChessSpecific.Bishop.java 0.1 2023/03/17
 *
 * Copyright (c) Omar Ibrahim
 * All rights reserved.
 */
package ChessSpecific;


/**
 * Bishop - Interactive Robot Chess Bishop Class that represents a bishop.
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
public class Bishop extends ChessPiece {
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
     * Constructor for creating a Bishop with a color.
     *
     * @param isWhite is color of the piece white
     */
    public Bishop(boolean isWhite){
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
    public String getName(){return "bishop";}

    /**
     * @return a single String value for the piece
     */
    public String toString(){
        if(isWhite) return "B";
        else return "b";
    }
}
