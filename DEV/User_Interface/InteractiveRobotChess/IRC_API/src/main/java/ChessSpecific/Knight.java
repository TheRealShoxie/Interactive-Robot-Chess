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
public class Knight extends ChessPiece {
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

    public Knight(boolean isWhite){
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
                MoveList.KNIGHT_LEFT_UP,
                MoveList.KNIGHT_UP_LEFT,
                MoveList.KNIGHT_UP_RIGHT,
                MoveList.KNIGHT_RIGHT_UP,
                MoveList.KNIGHT_RIGHT_DOWN,
                MoveList.KNIGHT_DOWN_RIGHT,
                MoveList.KNIGHT_DOWN_LEFT,
                MoveList.KNIGHT_LEFT_DOWN
        };
        return m;
    }

    public boolean usesSingleMove(){return true;}
    public String getName(){return "knight";}
}
