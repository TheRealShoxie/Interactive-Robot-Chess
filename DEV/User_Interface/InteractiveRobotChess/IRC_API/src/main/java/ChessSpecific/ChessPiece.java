/*
 *@(#) Utility.DataChecker.java 0.1 2023/03/17
 *
 * Copyright (c) Omar Ibrahim
 * All rights reserved.
 */
package ChessSpecific;

/**
 * ChessPiece - Gives basic Structure to all chess pieces
 * <p>
 * This class was written using Piece.java from https://github.com/Stevoisiak/JavaFX-Online-Chess
 * @author Omar Ibrahim
 * @version 0.1 ( Initial development ).
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
    protected boolean isWhite;

    // ///////////// //
    // Constructors. //
    // ///////////// //

    public ChessPiece(boolean isWhite){
        this.isWhite = isWhite;
    }

    // ////////////////////// //
    // Read/Write properties. //
    // ////////////////////// //

    // ///////////////////// //
    // Read-only properties. //
    // ///////////////////// //

    /*public String getImageFilePath() {
        return this.imageFilePath;
    }*/

    public String getColor() {
        if(this.isWhite) return "white";
        else return "black";
    }


    // //////// //
    // Methods. //
    // //////// //
    public abstract String getName();
}
