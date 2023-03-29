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

    protected boolean hasMoved;
    protected boolean isWhite;
    //protected String imageFilePath;

    // ///////////// //
    // Constructors. //
    // ///////////// //

    public ChessPiece(boolean isWhite){
        this.isWhite = isWhite;

        hasMoved = false;
    }

    // ////////////////////// //
    // Read/Write properties. //
    // ////////////////////// //

    public void setHasMoved(boolean hasMoved) {
        this.hasMoved = hasMoved;
    }

    public boolean isHasMoved() {
        return this.hasMoved;
    }


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

    public boolean isWhite(){
        return this.isWhite;
    }


    // //////// //
    // Methods. //
    // //////// //
    public abstract MoveList[] getChessPieceMove();
    public abstract String getName();
    public abstract boolean usesSingleMove();
}
