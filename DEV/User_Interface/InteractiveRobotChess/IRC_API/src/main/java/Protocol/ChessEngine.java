/*
 *@(#) Utility.DataChecker.java 0.1 2023/03/17
 *
 * Copyright (c) Omar Ibrahim
 * All rights reserved.
 */
package Protocol;

import Client.IRCClient;
import CustomException.ProtocolException;
import Enum.ProtocolErrors;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.util.Arrays;
import java.util.List;

/**
 * ClassName - ClassDescription initial
 * <p>
 * What it does
 * @author Omar Ibrahim
 * @version 0.1 ( Initial development ).
 */
public class ChessEngine {
    // ////////// //
    // Constants. //
    // ////////// //
    final static byte cmdByteGetChessEngines = (byte)0x04;
    final static byte cmdByteStartChessEngine = (byte)0x05;
    final static byte cmdByteStopChessEngine = (byte)0x06;
    final static byte chessEngineNotFound = (byte)0xF9;
    final static byte chessEngineNotStarted = (byte)0xF8;
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
    String FENPosition;
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
     * @return The possible chess engines to choose from of type List<String>
     */
    public List<String> getChessEngineChoices() {
        return chessEngineChoices;
    }


    // //////// //
    // Methods. //
    // //////// //

    /**
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
        // Command was not recognizable
        else if(receivedCommandByte == unrecognizableCmd) throw new ProtocolException(ProtocolErrors.UNRECOGNIZABLE_CMD.toString());
        // Does not match expected return
        else throw new ProtocolException(ProtocolErrors.UNEXPECTED_RETURN_CMD.toString());
    }


    /**
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
        // Command was not recognizable
        else if(receivedCommandByte == unrecognizableCmd) throw new ProtocolException(ProtocolErrors.UNRECOGNIZABLE_CMD.toString());
        // Does not match expected return
        else throw new ProtocolException(ProtocolErrors.UNEXPECTED_RETURN_CMD.toString());
    }

    public void makeChessMove(IRCClient ircClient){

        //Checking if move is a valid move
        if(true){

        }
    }


}
