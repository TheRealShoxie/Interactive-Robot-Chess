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
    private ChessPiece chessPiece;
    private int xPos;
    private int yPos;
    private boolean lightCell;

    // ///////////// //
    // Constructors. //
    // ///////////// //
    public Cell(boolean light, int x, int y){
        this.xPos = x;
        this.yPos = y;
        this.chessPiece = null;
        this.lightCell = light;
    }

    // ////////////////////// //
    // Read/Write properties. //
    // ////////////////////// //

    public void setChessPiece(ChessPiece chessPiece) {
        this.chessPiece = chessPiece;
    }

    public ChessPiece getChessPiece() {
        return chessPiece;
    }

    public int getXPos() {
        return xPos;
    }

    public int getYPos() {
        return yPos;
    }

    public boolean isLightCell() {
        return lightCell;
    }

    // ///////////////////// //
    // Read-only properties. //
    // ///////////////////// //

    // //////// //
    // Methods. //
    // //////// //

    public ChessPiece releaseChessPiece(){
        ChessPiece tempChessPiece = this.chessPiece;
        setChessPiece(null);
        return tempChessPiece;
    }
}
