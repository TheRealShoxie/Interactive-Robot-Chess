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
public class Rook extends ChessPiece {
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

    public Rook(boolean isWhite){
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
    public String getName(){return "rook";}
}
