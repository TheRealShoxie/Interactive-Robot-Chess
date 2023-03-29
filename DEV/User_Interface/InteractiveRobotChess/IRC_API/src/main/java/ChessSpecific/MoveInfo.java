/*
 *@(#) Utility.DataChecker.java 0.1 2023/03/17
 *
 * Copyright (c) Omar Ibrahim
 * All rights reserved.
 */
package ChessSpecific;

import java.io.Serializable;

/**
 * ClassName - ClassDescription initial
 * <p>
 * What it does
 * @author Omar Ibrahim
 * @version 0.1 ( Initial development ).
 */
public class MoveInfo implements Serializable {
    // ////////// //
    // Constants. //
    // ////////// //

    // //////////////// //
    // Class variables. //
    // //////////////// //

    // ////////////// //
    // Class methods. //
    // ////////////// //

    private String getCharLabel(int i){
        return i > 0 && i < 27 ? String.valueOf((char)(i+64)).toLowerCase() : null;
    }

    // /////////////////// //
    // Instance variables. //
    // /////////////////// //
    private Cell oldCell;
    private Cell newCell;

    // ///////////// //
    // Constructors. //
    // ///////////// //
    public MoveInfo(Cell oldCell, Cell newCell){
        this.oldCell = oldCell;
        this.newCell = newCell;
    }

    // ////////////////////// //
    // Read/Write properties. //
    // ////////////////////// //

    public Cell getOldCell(){ return oldCell; }
    public Cell getNewCell(){ return newCell; }

    // ///////////////////// //
    // Read-only properties. //
    // ///////////////////// //

    // //////// //
    // Methods. //
    // //////// //

    public int getGapX(){ return this.newCell.getXPos() - this.oldCell.getXPos(); }

    public int getGapY(){ return this.newCell.getYPos() - this.oldCell.getYPos(); }

    @Override
    public String toString(){
        return (getCharLabel(oldCell.getXPos()+1) + (oldCell.getYPos()+1) +
                getCharLabel(newCell.getXPos()+1) + (newCell.getYPos()+1));
    }
}
