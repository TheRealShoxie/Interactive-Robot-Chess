/*
 *@(#) Utility.DataChecker.java 0.1 2023/03/17
 *
 * Copyright (c) Omar Ibrahim
 * All rights reserved.
 */
package ibo1.UIElements;

import ChessSpecific.Cell;
import javafx.scene.control.Button;
import javafx.scene.image.Image;
import javafx.scene.image.ImageView;

import java.net.URL;


/**
 * ClassName - ClassDescription initial
 * <p>
 * What it does
 * @author Omar Ibrahim
 * @version 0.1 ( Initial development ).
 */
public class Space extends Button {
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
    Cell cell;

    // ///////////// //
    // Constructors. //
    // ///////////// //
    public Space(Cell cell){
        super();
        this.getStyleClass().add("chess-space");

        if (cell.isLightCell())
            this.getStyleClass().add("chess-space-light");
        else
            this.getStyleClass().add("chess-space-dark");

        setCell(cell);
        this.prefWidth(20);
        this.prefHeight(30);

    }

    // ////////////////////// //
    // Read/Write properties. //
    // ////////////////////// //

    public void setCell(Cell cell) {
        this.cell = cell;

        if(cell.getChessPiece() != null){
            String location ="../ChessPieces/";
            String filename = cell.getChessPiece().getColor() + "_" + cell.getChessPiece().getName() + ".png";
            URL urlOfImage = getClass().getResource(location+filename);
            this.setGraphic(new ImageView( new Image(urlOfImage.toString())));
        }
        else{
            this.setGraphic(new ImageView());
        }
    }


    // ///////////////////// //
    // Read-only properties. //
    // ///////////////////// //

    // //////// //
    // Methods. //
    // //////// //

}
