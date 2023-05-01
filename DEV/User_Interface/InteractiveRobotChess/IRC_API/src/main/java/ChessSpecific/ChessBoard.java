/*
 *@(#) ChessSpecific.ChessBoard.java 0.1 2023/04/30
 *
 * Copyright (c) Omar Ibrahim
 * All rights reserved.
 */
package ChessSpecific;

import Client.IRCClient;
import CustomException.InvalidDataException;
import CustomException.ProtocolException;
import Protocol.ChessEngine;

import java.io.IOException;


/**
 * ChessBoard - Interactive Robot Chess ChessBoard class is the internal representation of a chess board.
 * It uses the Protocol.ChessEngine class to perform player and chess-engine moves.
 * It uses all the other classes listed in the ChessSpecific package.
 * It creates a two-dimensional array 8x8 elements of type Cell. These cells can hold chess pieces.
 *
 * <p>
 * 3rd party code is used in this class. It is an adaptions from GitHub user: Stevoisiak.
 * Link to the original code: <a href="https://github.com/Stevoisiak/JavaFX-Online-Chess">Github Link</a>.
 * This class represents the ChessBoard.java file
 * Accessed 30th of April 2023
 *
 * <p>
 * Following methods are adaptation and reworked functionality from the original work:
 * <ul>
 *     <li>setStartPosition()</li>
 *     <li>ChessBoard()</li>
 * </ul>
 *
 *
 * @author Omar Ibrahim
 * @version 0.1 ( Initial development ).
 * @version 0.2 ( Second development ).
 * @version 1.0 ( Initial release ).
 *
 * @see IRCClient
 * @see InvalidDataException
 * @see ChessEngine
 * @see Cell
 * @see ChessPiece
 * @see Bishop
 * @see King
 * @see Knight
 * @see Pawn
 * @see Queen
 * @see Rook
 */
public class ChessBoard {
    // ////////// //
    // Constants. //
    // ////////// //

    // //////////////// //
    // Class variables. //
    // //////////////// //

    // ////////////// //
    // Class methods. //
    // ////////////// //

    /**
     * Sets the chessboard into its start position
     */
    private void setStartPosition(){

        // Creating temporary variables to use for decision on where pieces go
        int whiteY;
        int whitePawnY;
        int blackY;
        int blackPawnY;
        int kingPos;
        int queenPos;

        // Checking if the player is White
        if(this.playerIsWhite){
            whiteY = 7;
            whitePawnY = 6;
            blackY = 0;
            blackPawnY = 1;
            kingPos = 4;
            queenPos = 3;
        }

        // Otherwise the player is black
        else{
            whiteY = 0;
            whitePawnY = 1;
            blackY = 7;
            blackPawnY = 6;
            kingPos = 3;
            queenPos = 4;
        }

        // Filling the chessboard cells with the corresponding chess pieces
        //white pieces
        this.cells[0][whiteY].setChessPiece(new Rook(true));
        this.cells[1][whiteY].setChessPiece(new Knight(true));
        this.cells[2][whiteY].setChessPiece(new Bishop(true));
        this.cells[queenPos][whiteY].setChessPiece(new Queen(true));
        this.cells[kingPos][whiteY].setChessPiece(new King(true));
        this.cells[5][whiteY].setChessPiece(new Bishop(true));
        this.cells[6][whiteY].setChessPiece(new Knight(true));
        this.cells[7][whiteY].setChessPiece(new Rook(true));

        for(int i = 0; i < this.cells[0].length; i++)
            this.cells[i][whitePawnY].setChessPiece(new Pawn(true));

        //black pieces
        this.cells[0][blackY].setChessPiece(new Rook(false));
        this.cells[1][blackY].setChessPiece(new Knight(false));
        this.cells[2][blackY].setChessPiece(new Bishop(false));
        this.cells[queenPos][blackY].setChessPiece(new Queen(false));
        this.cells[kingPos][blackY].setChessPiece(new King(false));
        this.cells[5][blackY].setChessPiece(new Bishop(false));
        this.cells[6][blackY].setChessPiece(new Knight(false));
        this.cells[7][blackY].setChessPiece(new Rook(false));

        for(int i = 0; i < this.cells[0].length; i++)
            this.cells[i][blackPawnY].setChessPiece(new Pawn(false));
    }


    /**
     * Get a chessMoveFormat from initial cell to destination cell.
     * Example of chess move is: a1a2, c2c4
     *
     * @param initialCell Cell where the chess piece gets moved from
     * @param destinationCell Cell where the chess piece gets moved to
     * @param sb stringBuilder that gets manipulated to hold the chess move
     */
    private void convertToMoveFormat(Cell initialCell, Cell destinationCell, StringBuilder sb){

        // Temp variable to build the move command
        char[] moveCmd = new char[4];

        // Checking if we are white player as we need to convert the cells to chess move differently
        if(this.playerIsWhite){

            // Converting the cell positions to moveCommand
            moveCmd[0] = (char)(initialCell.getXPos()+97);
            moveCmd[1] = Character.forDigit(7-initialCell.getYPos()+1, 10);
            moveCmd[2] = (char)(destinationCell.getXPos()+97);
            moveCmd[3] = Character.forDigit(7-destinationCell.getYPos()+1, 10);
        }

        // Otherwise we are black player
        else{
            moveCmd[0] = (char)(7-initialCell.getXPos()+97);
            moveCmd[1] = Character.forDigit(initialCell.getYPos()+1, 10);
            moveCmd[2] = (char)(7-destinationCell.getXPos()+97);
            moveCmd[3] = Character.forDigit(destinationCell.getYPos()+1, 10);
        }

        // Manipulating the supplied string builder to add the moveCmd
        sb.append(String.valueOf(moveCmd));
    }


    /**
     * Converts a chess move to cell Position
     *
     * The return is a 4 element integer with first two elements being x and y for first cell
     * and element 3 and 4 being x and y for the second cell
     *
     * @param move chess move to convert
     * @return returns an array with 4 values
     */
    private int[] convertFromMoveFormat(String move){

        // Converting into a char array
        char[] moveCmd = move.toLowerCase().toCharArray();

        // Temp cell Positions
        int oldCellX = 0;
        int oldCellY = 0;
        int newCellX = 0;
        int newCellY = 0;

        // Checking if player is white
        if(this.playerIsWhite){

            // Converting the move command to cell positions
            oldCellX = (int)moveCmd[0] - 97;
            oldCellY = 8 - Character.getNumericValue(moveCmd[1]);
            newCellX = (int)moveCmd[2] - 97;
            newCellY = 8 - Character.getNumericValue(moveCmd[3]);

        }
        // otherwise we are player black
        else{

            oldCellX = 7 - ((int)moveCmd[0] - 97);
            oldCellY = Character.getNumericValue(moveCmd[1]) - 1;
            newCellX = 7 - ((int)moveCmd[2] - 97);
            newCellY = Character.getNumericValue(moveCmd[3]) - 1;
        }

        // Creating the temp array to return
        int returnedInt[] = new int[4];
        returnedInt[0] = oldCellX;
        returnedInt[1] = oldCellY;
        returnedInt[2] = newCellX;
        returnedInt[3] = newCellY;

        // Returning the cell positions
        return returnedInt;
    }

    /**
     * This method performs the rook move of the castle.
     *
     * @param moveFormat supplied chess move
     */
    private void castleMove(String moveFormat){

        // Checking if bottom color still has castle rights
        if (castleRights[0]){
            // Checking if the move was an e1g1 move
            if(moveFormat.equals("e1g1")){

                // Checking if the piece in the e1g1 cell is a king
                int cellPosKing[] = convertFromMoveFormat("e1g1");

                // Checking if the king is not there
                if(!(this.cells[cellPosKing[0]][cellPosKing[0]].getChessPiece().getName().toLowerCase().equals("k"))){
                    // Then we do not perform a castle move and should remove the castle rights
                    castleRights[0] = false;
                    return;
                }

                // Convert move format to cell pos for the rook
                int cellPosRook[] = convertFromMoveFormat("h1f1");

                // releasing the chess piece from the cell
                ChessPiece tempChessPieceRook = this.cells[cellPosRook[0]][cellPosRook[1]].releaseChessPiece();

                //Setting the chess piece in the new cell
                this.cells[cellPosRook[2]][cellPosRook[3]].setChessPiece(tempChessPieceRook );

                // Removing castle rights for this side
                castleRights[0] = false;
            }

            // Otherwise was the move an e1c1 move
            else if(moveFormat.equals("e1c1")){

                // Checking if the piece in the e1g1 cell is a king
                int cellPosKing[] = convertFromMoveFormat("e1g1");

                // Checking if the king is not there
                if(!(this.cells[cellPosKing[0]][cellPosKing[0]].getChessPiece().getName().toLowerCase().equals("k"))){
                    // Then we do not perform a castle move and should remove the castle rights
                    castleRights[0] = false;
                    return;
                }

                int cellPosRook[] = convertFromMoveFormat("a1d1");

                ChessPiece tempChessPieceRook = this.cells[cellPosRook[0]][cellPosRook[1]].releaseChessPiece();
                this.cells[cellPosRook[2]][cellPosRook[3]].setChessPiece(tempChessPieceRook );

                // Removing castle rights for this side
                castleRights[0] = false;

            }
        }

        // Otherwise checking if top color still has castle rights
        else if(castleRights[1]){
            // Checking if move was an e8g8 move
            if(moveFormat.equals("e8g8")){

                // Checking if the piece in the e1g1 cell is a king
                int cellPosKing[] = convertFromMoveFormat("e1g1");

                // Checking if the king is not there
                if(!(this.cells[cellPosKing[0]][cellPosKing[0]].getChessPiece().getName().toLowerCase().equals("k"))){
                    // Then we do not perform a castle move and should remove the castle rights
                    castleRights[1] = false;
                    return;
                }

                int cellPosRook[] = convertFromMoveFormat("h8f8");

                ChessPiece tempChessPieceRook = this.cells[cellPosRook[0]][cellPosRook[1]].releaseChessPiece();
                this.cells[cellPosRook[2]][cellPosRook[3]].setChessPiece(tempChessPieceRook );

                // Removing castle rights for this side
                castleRights[1] = false;
            }

            // Otherwise was the move an e8c8 move
            else if(moveFormat.equals("e8c8")){

                // Checking if the piece in the e1g1 cell is a king
                int cellPosKing[] = convertFromMoveFormat("e1g1");

                // Checking if the king is not there
                if(!(this.cells[cellPosKing[0]][cellPosKing[0]].getChessPiece().getName().toLowerCase().equals("k"))){
                    // Then we do not perform a castle move and should remove the castle rights
                    castleRights[1] = false;
                    return;
                }

                int cellPosRook[] = convertFromMoveFormat("a8d8");

                ChessPiece tempChessPieceRook = this.cells[cellPosRook[0]][cellPosRook[1]].releaseChessPiece();
                this.cells[cellPosRook[2]][cellPosRook[3]].setChessPiece(tempChessPieceRook );

                // Removing castle rights for this side
                castleRights[1] = false;
            }
        }




    }


    /**
     * This Function performce a promotion if it is one
     *
     * @param moveFormat provided chess move
     */
    private void promotion(String moveFormat){

        // Checking if length is not 5 which is a chess promotion move then return
        if(moveFormat.length() != 5) return;

        // Getting the chess move from the promotion move
        String move = moveFormat.substring(0,4);

        // Converting the move to cell positions
        int cellPos[] = convertFromMoveFormat(move);

        // Getting the last char of the string which represents the piece to promote into
        char toPromoteTo = moveFormat.toLowerCase().charAt(4);

        // Getting a temporary variable for the chessPiece to promote
        ChessPiece chessPiecePromoted;

        // Checking which piece we promote into
        switch(toPromoteTo){

            // Case rook
            case 'r':

                //Checking if the piece we promote from is white then create a white rook
                if(cells[cellPos[0]][cellPos[1]].getChessPiece().isWhite) chessPiecePromoted = new Rook(true);
                // Otherwise create a black rook
                else chessPiecePromoted = new Rook(false);
                break;

            // Case knight
            case 'n':
                //Checking if the piece we promote from is white then create a white knight
                if(cells[cellPos[0]][cellPos[1]].getChessPiece().isWhite) chessPiecePromoted = new Knight(true);
                // Otherwise create a black knight
                else chessPiecePromoted = new Knight(false);
                break;

            // Case bishop
            case 'b':
                //Checking if the piece we promote from is white then create a white bishop
                if(cells[cellPos[0]][cellPos[1]].getChessPiece().isWhite) chessPiecePromoted = new Bishop(true);
                // Otherwise create a black bishop
                else chessPiecePromoted = new Bishop(false);
                break;

            // Case queen
            case 'q':
                //Checking if the piece we promote from is white then create a white queen
                if(cells[cellPos[0]][cellPos[1]].getChessPiece().isWhite) chessPiecePromoted = new Queen(true);
                // Otherwise create a black queen
                else chessPiecePromoted = new Queen(false);
                break;

            // No default case
            default:
                return;
        }

        // Releasing the initial chesspiece
        this.cells[cellPos[0]][cellPos[1]].releaseChessPiece();

        // Placing the created promoted chess piece in the cell it should go to
        this.cells[cellPos[2]][cellPos[3]].setChessPiece( chessPiecePromoted );

    }

    // /////////////////// //
    // Instance variables. //
    // /////////////////// //

    /**
     * Cells that represent a chessBoard
     */
    Cell[][] cells;

    /**
     * If the player is white
     */
    boolean playerIsWhite;

    /**
     * Array that keeps track of the castle Rights
     */
    boolean[] castleRights;

    /**
     * The chess engine instance to be used for networking
     */
    ChessEngine chessEngine;

    /**
     * The IRCClient instance for networking
     */
    IRCClient ircClient;

    /**
     * Constructor for ChessBoard
     *
     * @param playerIsWhite if the player is white
     * @param chessEngine reference to the ChessEngine
     * @param ircClient reference to the IRCClient
     */
    // ///////////// //
    // Constructors. //
    // ///////////// //
    public ChessBoard(boolean playerIsWhite, ChessEngine chessEngine, IRCClient ircClient){

        //Setting up cell grid for the board
        this.cells = new Cell[8][8];
        this.playerIsWhite = playerIsWhite;
        this.chessEngine = chessEngine;
        this.ircClient = ircClient;
        this.castleRights = new boolean[2];
        castleRights[0] = true;
        castleRights[1] = true;

        // iterating through each cell to create the light and black squares
        for(int x = 0; x < cells[0].length; x++){
            for(int y = 0; y < cells[1].length; y++){
                Cell cell;

                //Checking if cell should be a light-colored cell or dark-colored
                if( (x+y)%2 != 0) cell = new Cell(false, x, y);
                else cell = new Cell(true, x, y);

                cells[x][y] = cell;
            }
        }

        // Setting the board up in start position
        this.setStartPosition();
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

    /**
     * Get a cell at a specific position
     *
     * @param x X position of the cell
     * @param y Y position of the cell
     * @return returns a copy of the cell
     */
    public Cell getCellAtPosition(int x, int y){
        return this.cells[x][y];
    }

    /**
     * Making a player move. This is done by calling the player move method in the
     * ChessEngine class.
     *
     * @param activeCellX startingCell X position
     * @param activeCellY startingCell Y position
     * @param clickedCellX destinationCell X position
     * @param clickedCellY destinationCell Y position
     * @param toPromoteInto add a value for promotion move
     * @return returns true if successfull otherwise false if we do not have a chess move or promotion move
     *
     * @throws ProtocolException thrown by makePlayerMove from the ChessEngine class
     * @throws IOException thrown by makePlayerMove from the ChessEngine class
     */
    public boolean playerMove(int activeCellX, int activeCellY, int clickedCellX, int clickedCellY, char toPromoteInto) throws ProtocolException, IOException {

        // Creating a string builder and then converting the cell positions into a chessmove
        StringBuilder move = new StringBuilder();
        convertToMoveFormat(this.cells[activeCellX][activeCellY], this.cells[clickedCellX][clickedCellY], move);

        // Checking if we got supplied a promotion move
        if(toPromoteInto != ' '){
            move.append(toPromoteInto);
        }

        System.out.println("Sending player move with chess move: ");

        chessEngine.makePlayerMove(ircClient, move.toString());

        // If we successfully made the player move we check if the length of the move is equal 4
        if(move.toString().length() == 4){

            //If yes try to do a castleMove()
            castleMove(move.toString());

            //Make the move by releasing the chess piece from the initial cells position
            ChessPiece tempChessPiece = this.cells[activeCellX][activeCellY].releaseChessPiece();

            //Put the chess piece into the destination cell position
            this.cells[clickedCellX][clickedCellY].setChessPiece(tempChessPiece );
        }

        //Otherwise we check if it is a promotion move
        else if(move.toString().length() == 5){

            // Make a promotion move
            promotion(move.toString());
        }
        //Otherwise we do not have a promotion or chess move thus return false
        else{
            return false;
        }

        return true;
    }


    /**
     * Making a chess engine move. This is done by calling the chessEngineMove method in the
     * ChessEngine class.
     *
     * @throws ProtocolException thrown by makePlayerMove from the ChessEngine class
     * @throws IOException thrown by makePlayerMove from the ChessEngine class
     */
    public void chessEngineMove() throws ProtocolException, IOException {

        // Temp variable to hold the chess engine move
        String chessEngineMoved = "";

        // Calls the makeChessEngine move
        chessEngineMoved = chessEngine.getChessEngineMove(ircClient);

        // If makeChessEngineMove didn't throw an error we will get here
        System.out.println("Received chess engine move: " +chessEngineMoved);

        // Checking if returned move is length 4, if yes we deal with a normal chess engine move
        if(chessEngineMoved.length() == 4){

            // Convert the move to cell positions
            int cellPos[] = convertFromMoveFormat(chessEngineMoved);

            // Make a castle move if it is one
            castleMove(chessEngineMoved);

            //Moving the chess piece
            ChessPiece tempChessPiece = this.cells[cellPos[0]][cellPos[1]].releaseChessPiece();
            this.cells[cellPos[2]][cellPos[3]].setChessPiece(tempChessPiece );
        }

        // Otherwise checking if it is a promotion move then make a promotion move
        else if(chessEngineMoved.length() == 5){

            // Make a promotion move
            promotion(chessEngineMoved);
        }


    }

}
