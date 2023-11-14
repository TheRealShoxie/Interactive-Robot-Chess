/*
 *@(#) ibo1.Protocol.ChessEngine.java 0.1 2023/03/17
 *
 */
package ibo1.Protocol;

import ibo1.Client.IRCClient;
import ibo1.CustomException.ProtocolException;
import ibo1.Enum.ProtocolErrors;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.util.Arrays;
import java.util.List;

/**
 * ChessEngine - Class that represents ChessEngine communications
 * <p>
 * It defines the commands and errors and communication with the ROS system for chess moves.
 * Direct representation would be the ChessWrapperNode.cpp in the ROS system.
 *
 * @author Omar Ibrahim
 * @version 0.1 ( Initial development ).
 * @version 1.0 ( Initial release ).
 *
 * @see ibo1.ChessSpecific.ChessBoard
 * @see ProtocolException
 * @see ProtocolErrors
 * @see IRCClient
 *
 */
public class ChessEngine {

    // ////////// //
    // Constants. //
    // ////////// //
    /**
     * CMD byte from the protocol
     */
    final static byte cmdByteGetChessEngines = (byte)0x04;

    /**
     * CMD byte from the protocol
     */
    final static byte cmdByteStartChessEngine = (byte)0x05;

    /**
     * CMD byte from the protocol
     */
    final static byte cmdByteStopChessEngine = (byte)0x06;

    /**
     * CMD byte from the protocol
     */
    final static byte cmdByteGetChessEngineOptions = (byte)0x07;

    /**
     * CMD byte from the protocol
     */
    final static byte cmdByteSetChessEngineOption = (byte)0x08;

    /**
     * CMD byte from the protocol
     */
    final static byte cmdByteSetSearchEngineOption = (byte)0x09;

    /**
     * CMD byte from the protocol
     */
    final static byte cmdBytePlayerMove = (byte)0x0A;

    /**
     * CMD byte from the protocol
     */
    final static byte cmdByteChessEngineMove = (byte)0x0B;



    /**
     * Error byte from the protocol
     */
    final static byte chessEngineNotFound = (byte)0xF9;

    /**
     * Error byte from the protocol
     */
    final static byte chessEngineNotStarted = (byte)0xF8;

    /**
     * Error byte from the protocol
     */
    final static byte chessEngineNotRunning = (byte)0xF7;

    /**
     * Error byte from the protocol
     */
    final static byte chessEngineOptionDoesntExist = (byte)0xF6;

    /**
     * Error byte from the protocol
     */
    final static byte chessEngineNoSuchSearchOption = (byte)0xF5;

    /**
     * Error byte from the protocol
     */
    final static byte chessEnginePawnCollidedStraight = (byte)0xF4;

    /**
     * Error byte from the protocol
     */
    final static byte chessEnginePawnCollidedDiagonalOrEmptySpace = (byte)0xF3;

    /**
     * Error byte from the protocol
     */
    final static byte chessEngineEmptyStartingCell = (byte)0xF2;

    /**
     * Error byte from the protocol
     */
    final static byte chessEngineNotThatColorsTurn = (byte)0xF1;

    /**
     * Error byte from the protocol
     */
    final static byte chessEngineMoveInvalidOrBlockedBySameColor = (byte)0xF0;

    /**
     * Error byte from the protocol
     */
    final static byte chessEngineCannotCastleKingSide = (byte)0xEF;

    /**
     * Error byte from the protocol
     */
    final static byte chessEngineCannotCastleQueenSide = (byte)0xEE;

    /**
     * Error byte from the protocol
     */
    final static byte chessEngineOwnKingInCheck = (byte)0xED;

    /**
     * Error byte from the protocol
     */
    final static byte chessEngineOtherKingInCheckMate = (byte)0xEC;

    /**
     * Error byte from the protocol
     */
    final static byte chessEnginePawnNotAllowedIntoPositionNoPromotionMove = (byte)0xEB;

    /**
     * Error byte from the protocol
     */
    final static byte chessEnginePieceToPromoteNotPawn = (byte)0xEA;

    /**
     * Error byte from the protocol
     */
    final static byte chessEnginePawnNotMovingIntoEndPos = (byte)0xE9;

    /**
     * Error byte from the protocol
     */
    final static byte chessEngineInvalidNameForPieceToPromoteInto = (byte)0xE8;

    /**
     * Error byte from the protocol
     */
    final static byte chessEngineCreatedNoMove = (byte)0xE7;

    /**
     * Error byte from the protocol
     */
    final static byte chessEngineMoveFormatInvalid = (byte)0xE6;

    /**
     * Error byte from the protocol
     */
    final static byte noChessBoardInformationFromCameraPublished = (byte)0xE5;

    /**
     * Error byte from the protocol
     */
    final static byte pickUpCellEmptyInternalAndRealWorldChessBoardMismatch = (byte)0xE4;

    /**
     * Error byte from the protocol
     */
    final static byte systemInPlayingChessStateMachine = (byte)0xE3;

    /**
     * Error byte from the protocol
     */
    final static byte unrecognizableCmd = (byte)0xFE;

    // //////////////// //
    // Class variables. //
    // //////////////// //


    // ////////////// //
    // Class methods. //
    // ////////////// //

    /**
     * Sets the internal chessEngineChoices to the data received from the protocol
     * command getChessEngines.
     *
     * @param data Received bytes from the protocol
     */
    private void createChessEngineOptionsList(byte[] data){
        String dataAsString = new String(data, StandardCharsets.UTF_8);
        String[] possibleChessEngines = dataAsString.split("\u241f");

        chessEngineChoices = Arrays.asList(possibleChessEngines);
    }



    // /////////////////// //
    // Instance variables. //
    // /////////////////// //


    /**
     * Holds the possible choices of chess engines to start.
     * Starts empty but is populated after the first getChessEngines request to the server
     */
    List<String> chessEngineChoices;


    // ///////////// //
    // Constructors. //
    // ///////////// //

    // ////////////////////// //
    // Read/Write properties. //
    // ////////////////////// //

    // ///////////////////// //
    // Read-only properties. //
    // ///////////////////// //

    /**
     * @return The possible chess engines to choose from of type List Of String
     */
    public List<String> getChessEngineChoices() {
        return chessEngineChoices;
    }



    // //////// //
    // Methods. //
    // //////// //

    /**
     * Gets the possible chess engines to play against from the ROS system-
     *
     * @param ircClient ircClient to be used
     * @throws IOException thrown by sending and receiving the buffer
     * @throws ProtocolException thrown if returned cmd does not match expected
     */
    public void getPossibleChessEngines(IRCClient ircClient) throws IOException, ProtocolException {

        // Creating the Protocol object for the possible chess Engine command
        ProtocolObject protocolGetChessEngines = new ProtocolObject();
        protocolGetChessEngines.setCmdByte(cmdByteGetChessEngines);

        // Sending the command
        ircClient.send(protocolGetChessEngines);

        // Retrieving the answer
        ProtocolObject receivedProtocol = ircClient.receive();

        byte receivedCommandByte = receivedProtocol.getCmdByte();

        // Setting the List options for chessEngines;
        if(receivedCommandByte == cmdByteGetChessEngines){
            // Setting chessEngineChoices
            createChessEngineOptionsList(receivedProtocol.getData());
        }
        else if(receivedCommandByte == systemInPlayingChessStateMachine) throw new ProtocolException(ProtocolErrors.SYSTEM_ALREADY_IN_CHESS_STATE.toString());
        // Command was not recognizable
        else if(receivedCommandByte == unrecognizableCmd) throw new ProtocolException(ProtocolErrors.UNRECOGNIZABLE_CMD.toString());
        // Does not match expected return
        else throw new ProtocolException(ProtocolErrors.UNEXPECTED_RETURN_CMD.toString());
    }


    /**
     * Starts the chess engine with the supplied chessEngine name on the ROS system
     *
     * @param ircClient ircClient to be used
     * @param chessEngineName chess engine to be started
     * @throws IOException thrown by sending and receiving the buffer
     * @throws ProtocolException thrown if returned cmd does not match expected or if chess engine doesn't exist.
     */
    public void startChessEngine(IRCClient ircClient, String chessEngineName) throws IOException, ProtocolException {

        byte[] dataStringAsBytes = chessEngineName.getBytes(StandardCharsets.UTF_8);

        // Creating the Protocol object for the possible chess Engine command
        ProtocolObject protocolStartChessEngine = new ProtocolObject();
        protocolStartChessEngine.setCmdByte(cmdByteStartChessEngine);
        protocolStartChessEngine.setDataSize(dataStringAsBytes.length);
        protocolStartChessEngine.setData(dataStringAsBytes);

        // Sending the command
        ircClient.send(protocolStartChessEngine);

        // Retrieving the answer
        ProtocolObject receivedProtocol = ircClient.receive();

        byte receivedCommandByte = receivedProtocol.getCmdByte();

        // Setting the List options for chessEngines;
        if(receivedCommandByte == cmdByteStartChessEngine){
            // Setting chessEngineChoices
            return;
        }
        else if(receivedCommandByte == chessEngineNotFound) throw new ProtocolException(ProtocolErrors.CHESSENGINE_NOT_FOUND.toString());
        else if(receivedCommandByte == chessEngineNotStarted) throw new ProtocolException(ProtocolErrors.CHESSENGINE_NOT_STARTED.toString());
        else if(receivedCommandByte == systemInPlayingChessStateMachine) throw new ProtocolException(ProtocolErrors.SYSTEM_ALREADY_IN_CHESS_STATE.toString());
        // Command was not recognizable
        else if(receivedCommandByte == unrecognizableCmd) throw new ProtocolException(ProtocolErrors.UNRECOGNIZABLE_CMD.toString());
        // Does not match expected return
        else throw new ProtocolException(ProtocolErrors.UNEXPECTED_RETURN_CMD.toString());
    }


    /**
     * Sends a player move to the ROS system
     *
     * @param ircClient ircClient to be used
     * @param moveCommand the player chess move to send
     * @throws IOException thrown by sending and receiving the buffer
     * @throws ProtocolException thrown if returned cmd does not match expected or if chess engine doesn't exist.
     */
    public void makePlayerMove(IRCClient ircClient, String moveCommand) throws IOException, ProtocolException {

        byte[] dataStringAsBytes = moveCommand.getBytes(StandardCharsets.UTF_8);

        // Creating the Protocol object for the possible chess Engine command
        ProtocolObject protocolPlayerMove = new ProtocolObject();
        protocolPlayerMove.setCmdByte(cmdBytePlayerMove);
        protocolPlayerMove.setDataSize(dataStringAsBytes.length);
        protocolPlayerMove.setData(dataStringAsBytes);

        // Sending the command
        ircClient.send(protocolPlayerMove);

        // Retrieving the answer
        ProtocolObject receivedProtocol = ircClient.receive();

        byte receivedCommandByte = receivedProtocol.getCmdByte();

        // Checking what byte we got returned
        switch (receivedCommandByte){
            case cmdBytePlayerMove:
                break;
            case chessEngineNotRunning:
                throw new ProtocolException(ProtocolErrors.CHESSENGINE_NOT_RUNNING.toString());

            case chessEnginePawnCollidedStraight:
                throw new ProtocolException(ProtocolErrors.CHESSENGINE_PAWN_COLLIDED_STRAIGHT.toString());

            case chessEnginePawnCollidedDiagonalOrEmptySpace:
                throw new ProtocolException(ProtocolErrors.CHESSENGINE_PAWN_COLLIDED_DIAGONAL_OREMPTYCELLTHERE.toString());

            case chessEngineEmptyStartingCell:
                throw new ProtocolException(ProtocolErrors.CHESSENGINE_STARTING_CELL_EMPTY.toString());

            case chessEngineNotThatColorsTurn:
                throw new ProtocolException(ProtocolErrors.CHESSENGINE_NOT_THAT_COLORS_TURN.toString());

            case chessEngineMoveInvalidOrBlockedBySameColor:
                throw new ProtocolException(ProtocolErrors.CHESSENGINE_MOVE_INVALID_OR_BLOCKED_BY_OWN_COLOR.toString());

            case chessEngineCannotCastleKingSide:
                throw new ProtocolException(ProtocolErrors.CHESSENGINE_CANNOT_CASTLE_KING_SIDE.toString());

            case chessEngineCannotCastleQueenSide:
                throw new ProtocolException(ProtocolErrors.CHESSENGINE_CANNOT_CASTLE_QUEEN_SIDE.toString());

            case chessEngineOwnKingInCheck:
                throw new ProtocolException(ProtocolErrors.CHESSENGINE_OWN_KING_IN_CHECK.toString());

            case chessEngineOtherKingInCheckMate:
                throw new java.net.ProtocolException(ProtocolErrors.CHESSENGINE_OTHER_KING_IN_CHECK_MATE.toString());

            case chessEnginePawnNotAllowedIntoPositionNoPromotionMove:
                throw new ProtocolException(ProtocolErrors.CHESSENGINE_PAWN_NOT_ALLOWED_NOT_PROMOTION_MOVE.toString());

            case chessEnginePieceToPromoteNotPawn:
                throw new ProtocolException(ProtocolErrors.CHESSENGINE_PIECE_TO_PROMOTE_NOT_PAWN.toString());

            case chessEnginePawnNotMovingIntoEndPos:
                throw new ProtocolException(ProtocolErrors.CHESSENGINE_PAWN_NOT_MOVING_INTO_END_POS.toString());

            case chessEngineInvalidNameForPieceToPromoteInto:
                throw new ProtocolException(ProtocolErrors.CHESSENGINE_INVALID_NAME_FOR_PIECE_TO_PROMOTE_INTO.toString());

            case chessEngineMoveFormatInvalid:
                throw new ProtocolException(ProtocolErrors.CHESSENGINE_MOVE_FORMAT_INVALID.toString());

            case noChessBoardInformationFromCameraPublished:
                throw new ProtocolException(ProtocolErrors.CAMERA_DOESNT_PUBLISH_INFO.toString());

            case pickUpCellEmptyInternalAndRealWorldChessBoardMismatch:
                throw new java.net.ProtocolException(ProtocolErrors.INTERNAL_REAL_CHESS_BOARD_MISMATCH.toString());

            case systemInPlayingChessStateMachine:
                throw new ProtocolException(ProtocolErrors.SYSTEM_ALREADY_IN_CHESS_STATE.toString());

                // Command was not recognizable
            case unrecognizableCmd :
                throw new ProtocolException(ProtocolErrors.UNRECOGNIZABLE_CMD.toString());

                // Does not match expected return
            default:
                throw new ProtocolException(ProtocolErrors.UNEXPECTED_RETURN_CMD.toString());
        }

    }


    /**
     * Gets the chess engine move from the ROS system
     *
     * @param ircClient ircClient to be used
     * @return the chess move from the engine if nothing went wrong
     * @throws IOException thrown by sending and receiving the buffer
     * @throws ProtocolException thrown if returned cmd does not match expected or if chess engine doesn't exist.
     */
    public String getChessEngineMove(IRCClient ircClient) throws IOException, ProtocolException {


        // Creating the Protocol object for the possible chess Engine command
        // Creating the Protocol object for the possible chess Engine command
        ProtocolObject protocolChessEngineMove = new ProtocolObject();
        protocolChessEngineMove.setCmdByte(cmdByteChessEngineMove);

        System.err.println("chessEngineMove:");
        System.out.println(protocolChessEngineMove.toString());

        // Sending the command
        ircClient.send(protocolChessEngineMove);

        // Retrieving the answer
        ProtocolObject receivedProtocol = ircClient.receive();

        byte receivedCommandByte = receivedProtocol.getCmdByte();


        // Checking what byte we got returned
        switch (receivedCommandByte){
            case cmdByteChessEngineMove:
                return new String(receivedProtocol.getData(), StandardCharsets.UTF_8);

            case chessEngineNotRunning:
                throw new ProtocolException(ProtocolErrors.CHESSENGINE_NOT_RUNNING.toString());

            case chessEnginePawnCollidedStraight:
                throw new ProtocolException(ProtocolErrors.CHESSENGINE_PAWN_COLLIDED_STRAIGHT.toString());

            case chessEnginePawnCollidedDiagonalOrEmptySpace:
                throw new ProtocolException(ProtocolErrors.CHESSENGINE_PAWN_COLLIDED_DIAGONAL_OREMPTYCELLTHERE.toString());

            case chessEngineEmptyStartingCell:
                throw new ProtocolException(ProtocolErrors.CHESSENGINE_STARTING_CELL_EMPTY.toString());

            case chessEngineNotThatColorsTurn:
                throw new ProtocolException(ProtocolErrors.CHESSENGINE_NOT_THAT_COLORS_TURN.toString());

            case chessEngineMoveInvalidOrBlockedBySameColor:
                throw new ProtocolException(ProtocolErrors.CHESSENGINE_MOVE_INVALID_OR_BLOCKED_BY_OWN_COLOR.toString());

            case chessEngineCannotCastleKingSide:
                throw new ProtocolException(ProtocolErrors.CHESSENGINE_CANNOT_CASTLE_KING_SIDE.toString());

            case chessEngineCannotCastleQueenSide:
                throw new ProtocolException(ProtocolErrors.CHESSENGINE_CANNOT_CASTLE_QUEEN_SIDE.toString());

            case chessEngineOwnKingInCheck:
                throw new ProtocolException(ProtocolErrors.CHESSENGINE_OWN_KING_IN_CHECK.toString());

            case chessEngineOtherKingInCheckMate:
                throw new java.net.ProtocolException(ProtocolErrors.CHESSENGINE_OTHER_KING_IN_CHECK_MATE.toString());

            case chessEnginePawnNotAllowedIntoPositionNoPromotionMove:
                throw new ProtocolException(ProtocolErrors.CHESSENGINE_PAWN_NOT_ALLOWED_NOT_PROMOTION_MOVE.toString());

            case chessEnginePieceToPromoteNotPawn:
                throw new ProtocolException(ProtocolErrors.CHESSENGINE_PIECE_TO_PROMOTE_NOT_PAWN.toString());

            case chessEnginePawnNotMovingIntoEndPos:
                throw new ProtocolException(ProtocolErrors.CHESSENGINE_PAWN_NOT_MOVING_INTO_END_POS.toString());

            case chessEngineInvalidNameForPieceToPromoteInto:
                throw new ProtocolException(ProtocolErrors.CHESSENGINE_INVALID_NAME_FOR_PIECE_TO_PROMOTE_INTO.toString());

            case chessEngineCreatedNoMove:
                throw new ProtocolException(ProtocolErrors.CHESSENGINE_CREATED_NO_MOVE.toString());

            case chessEngineMoveFormatInvalid:
                throw new ProtocolException(ProtocolErrors.CHESSENGINE_MOVE_FORMAT_INVALID.toString());

            case noChessBoardInformationFromCameraPublished:
                throw new ProtocolException(ProtocolErrors.CAMERA_DOESNT_PUBLISH_INFO.toString());

            case pickUpCellEmptyInternalAndRealWorldChessBoardMismatch:
                throw new java.net.ProtocolException(ProtocolErrors.INTERNAL_REAL_CHESS_BOARD_MISMATCH.toString());

            case systemInPlayingChessStateMachine:
                throw new ProtocolException(ProtocolErrors.SYSTEM_ALREADY_IN_CHESS_STATE.toString());

                // Command was not recognizable
            case unrecognizableCmd :
                throw new ProtocolException(ProtocolErrors.UNRECOGNIZABLE_CMD.toString());


                // Does not match expected return
            default:
                throw new ProtocolException(ProtocolErrors.UNEXPECTED_RETURN_CMD.toString());
        }
    }


    /**
     * Stops the chess engine. This is used to abort a game.
     * If a game is not stopped it will currently not work to start a new game.
     *
     * @param ircClient ircClient to be used
     * @throws IOException thrown by sending and receiving the buffer
     * @throws ProtocolException thrown if returned cmd does not match expected or if chess engine doesn't exist.
     */
    public void stopChessEngine(IRCClient ircClient) throws IOException, ProtocolException {

        // Creating the Protocol object for the possible chess Engine command
        ProtocolObject protocolSetNoRoboter = new ProtocolObject();
        protocolSetNoRoboter.setCmdByte(cmdByteStopChessEngine);

        // Sending the command
        ircClient.send(protocolSetNoRoboter);

        // Retrieving the answer
        ProtocolObject receivedProtocol = ircClient.receive();

        byte receivedCommandByte = receivedProtocol.getCmdByte();

        // Returning that we successfully set it;
        if(receivedCommandByte == cmdByteStopChessEngine){
            // Setting chessEngineChoices
            return;
        }
        else if(receivedCommandByte == chessEngineNotRunning) throw new ProtocolException(ProtocolErrors.CHESSENGINE_NOT_RUNNING.toString());
        else if(receivedCommandByte == systemInPlayingChessStateMachine) throw new ProtocolException(ProtocolErrors.SYSTEM_ALREADY_IN_CHESS_STATE.toString());
            // Command was not recognizable
        else if(receivedCommandByte == unrecognizableCmd) throw new ProtocolException(ProtocolErrors.UNRECOGNIZABLE_CMD.toString());
            // Does not match expected return
        else throw new ProtocolException(ProtocolErrors.UNEXPECTED_RETURN_CMD.toString());
    }


}
