/*
 *@(#) UIElements.Space.java 0.1 2023/03/17
 *
 * Copyright (c) Omar Ibrahim
 * All rights reserved.
 */
package ibo1.UIElements;

import ChessSpecific.*;
import javafx.scene.control.Button;
import javafx.scene.image.Image;
import javafx.scene.image.ImageView;

import java.net.URL;


/**
 * Space - Interactive Robot Chess Space class is the UI representation of a chess board cell
 * <p>
 * This class extends the Button element from JavaFX.
 * To display the information from a cell.
 *
 *
 * <p>
 * 3rd party code is used in this class. It is an adaptions from GitHub user: Stevoisiak.
 * Link to the original code: <a href="https://github.com/Stevoisiak/JavaFX-Online-Chess">Github Link</a>
 * This class represents the Space.java file
 *
 *
 *
 * @author Omar Ibrahim
 * @version 0.1 ( Initial development ).
 * @version 0.2 ( Second development ).
 * @version 1.0 ( Initial release ).
 *
 * @see ImageView
 * @see Button
 * @see Cell
 * @see ChessPiece
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


    /**
     * Constructor for Space. Takes a Cell from the IRC_API
     *
     * @param cell IRC_API Cell
     */
    public Space(Cell cell){

        // Calls the button javaFX super to initialize
        super();

        // Adds the styling described in chess-space
        this.getStyleClass().add("chess-space");

        // Checks if the cell is light or dark and uses that styling
        if (cell.isLightCell())
            this.getStyleClass().add("chess-space-light");
        else
            this.getStyleClass().add("chess-space-dark");

        // Sets the cell supplied
        setCell(cell);
        this.prefWidth(20);
        this.prefHeight(30);

    }

    // ////////////////////// //
    // Read/Write properties. //
    // ////////////////////// //

    /**
     * Method used for setting an IRC_API Cell
     *
     * @param cell the cell to set
     */
    public void setCell(Cell cell) {
        this.cell = cell;

        // Checks if the cell piece is not empty
        if(cell.getChessPiece() != null){

            // If not empty get the png for that chessPiece using its color and name and set it as the graphic
            String location ="../ChessPieces/";
            String filename = cell.getChessPiece().getColor() + "_" + cell.getChessPiece().getName() + ".png";
            URL urlOfImage = getClass().getResource(location+filename);
            this.setGraphic(new ImageView( new Image(urlOfImage.toString())));
        }

        // Otherwise set an empty graphic
        else{
            this.setGraphic(new ImageView());
        }
    }

    /**
     * Gets the current cell
     *
     * @return the IRC_API cell of the space
     */
    public Cell getCell(){
        return this.cell;
    }


    // ///////////////////// //
    // Read-only properties. //
    // ///////////////////// //

    // //////// //
    // Methods. //
    // //////// //

}
