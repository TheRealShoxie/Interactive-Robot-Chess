/*
 *@(#) Utility.DataChecker.java 0.1 2023/03/17
 *
 * Copyright (c) Omar Ibrahim
 * All rights reserved.
 */
package ChessSpecific;

import java.util.ArrayList;

/**
 * ClassName - ClassDescription initial
 * <p>
 * What it does
 * @author Omar Ibrahim
 * @version 0.1 ( Initial development ).
 */
public class Pawn extends ChessPiece {
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

    public Pawn(boolean isWhite){
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

        /*
         * Pawn movement is HIGHLY conditional, so this branches.
         * The list ensures correct direction and two-space movement.
         * All the board-dependent things (like diagonal iff capturing) are ChessBoard's job.
         */

        //braces ensure toArray() works later, see ArrayList docs for why
        MoveList[] moves = {};

        //since pawns will never be white AND black, only returns moves of correct direction
        if(this.isWhite)
        {
            ArrayList<MoveList> whiteMoves = new ArrayList<MoveList>();

            //standard straight, can't capture using this
            whiteMoves.add(MoveList.UP);

            //diagonals, can and must capture using this
            whiteMoves.add(MoveList.UP_RIGHT);
            whiteMoves.add(MoveList.UP_LEFT);

            //if hasn't moved, UP is valid board move, can't capture using this
            if(!hasMoved) {whiteMoves.add(MoveList.DOUBLE_UP);}

            moves = whiteMoves.toArray(moves);
        }
        else
        {
            ArrayList<MoveList> blackMoves = new ArrayList<MoveList>();

            //standard straight, can't capture
            blackMoves.add(MoveList.DOWN);

            //diagonals, can and must capture using this
            blackMoves.add(MoveList.DOWN_RIGHT);
            blackMoves.add(MoveList.DOWN_LEFT);

            //If hasn't moved, DOWN is valid board move, can't capture using this
            if(!hasMoved) {blackMoves.add(MoveList.DOUBLE_DOWN);}

            moves = blackMoves.toArray(moves);
        }

        return moves;
    }

    public boolean usesSingleMove(){return true;}
    public String getName(){return "pawn";}
}
