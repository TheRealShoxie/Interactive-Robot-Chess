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
    @Override
    public MoveList[] getChessPieceMove() {
        MoveList[] m ={
                MoveList.UP_RIGHT,
                MoveList.DOWN_RIGHT,
                MoveList.DOWN_LEFT,
                MoveList.UP_LEFT
        };
        return m;
    }

    public boolean usesSingleMove(){return false;}
    public String getName(){return "bishop";}
}