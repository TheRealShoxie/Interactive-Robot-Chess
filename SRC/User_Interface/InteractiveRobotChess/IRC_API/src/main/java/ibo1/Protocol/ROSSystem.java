/*
 *@(#) ibo1.Protocol.ROSSystem.java 0.1 2023/03/17
 *
 */

package ibo1.Protocol;

import ibo1.Client.IRCClient;
import ibo1.CustomException.ProtocolException;
import ibo1.Enum.ProtocolErrors;

import java.io.IOException;
import java.util.Arrays;
import java.util.List;

/**
 * ROSSystem - Class that represents ROSSystem communications
 * <p>
 * It defines the commands and errors and communication with the ROS system for the setting of system states in the
 * system state machine.
 * Direct representation would be the SystemStateMachine.cpp in the ROS system.
 *
 * @author Omar Ibrahim
 * @version 0.1 ( Initial development ).
 * @version 1.0 ( Initial release ).
 *
 * @see ProtocolException
 * @see ProtocolErrors
 * @see IRCClient
 *
 */
public class ROSSystem {
    // ////////// //
    // Constants. //
    // ////////// //

    /**
     * CMD byte from the protocol
     */
    final static byte cmdByteSetSystemWithoutRoboter = (byte)0x0C;

    /**
     * CMD byte from the protocol
     */
    final static byte cmdByteSetSystemFullSim = (byte)0x0D;


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
     * Sets the ROS system into no roboter arm state
     *
     * @param ircClient ircClient to be used
     * @throws IOException thrown by sending and receiving the buffer
     * @throws ProtocolException thrown if returned cmd does not match expected or if chess engine doesn't exist.
     */
    private void setSystemNoRoboter(IRCClient ircClient) throws IOException, ProtocolException {

        // Creating the Protocol object for the possible chess Engine command
        ProtocolObject protocolSetNoRoboter = new ProtocolObject();
        protocolSetNoRoboter.setCmdByte(cmdByteSetSystemWithoutRoboter);

        // Sending the command
        ircClient.send(protocolSetNoRoboter);

        // Retrieving the answer
        ProtocolObject receivedProtocol = ircClient.receive();

        byte receivedCommandByte = receivedProtocol.getCmdByte();

        // Returning that we successfully set it;
        if(receivedCommandByte == cmdByteSetSystemWithoutRoboter){
            // Setting chessEngineChoices
            return;
        }
        else if(receivedCommandByte == systemInPlayingChessStateMachine) throw new ProtocolException(ProtocolErrors.SYSTEM_ALREADY_IN_CHESS_STATE.toString());
            // Command was not recognizable
        else if(receivedCommandByte == unrecognizableCmd) throw new ProtocolException(ProtocolErrors.UNRECOGNIZABLE_CMD.toString());
            // Does not match expected return
        else throw new ProtocolException(ProtocolErrors.UNEXPECTED_RETURN_CMD.toString());
    }

    /**
     * Sets the ROS system into Full simulation
     *
     * @param ircClient ircClient to be used
     * @throws IOException thrown by sending and receiving the buffer
     * @throws ProtocolException thrown if returned cmd does not match expected or if chess engine doesn't exist.
     */
    private void setSystemInFullSimulationState(IRCClient ircClient) throws IOException, ProtocolException {

        // Creating the Protocol object for the possible chess Engine command
        ProtocolObject protocolSetFullSim = new ProtocolObject();
        protocolSetFullSim.setCmdByte(cmdByteSetSystemFullSim);

        // Sending the command
        ircClient.send(protocolSetFullSim);

        // Retrieving the answer
        ProtocolObject receivedProtocol = ircClient.receive();

        byte receivedCommandByte = receivedProtocol.getCmdByte();

        // Returning that we successfully set it;
        if(receivedCommandByte == cmdByteSetSystemFullSim){
            // Setting chessEngineChoices
            return;
        }
        else if(receivedCommandByte == systemInPlayingChessStateMachine) throw new ProtocolException(ProtocolErrors.SYSTEM_ALREADY_IN_CHESS_STATE.toString());
            // Command was not recognizable
        else if(receivedCommandByte == unrecognizableCmd) throw new ProtocolException(ProtocolErrors.UNRECOGNIZABLE_CMD.toString());
            // Does not match expected return
        else throw new ProtocolException(ProtocolErrors.UNEXPECTED_RETURN_CMD.toString());
    }

    // /////////////////// //
    // Instance variables. //
    // /////////////////// //

    /**
     * Possible systems states to choose from.
     * Current version only allows full simulation or no robot arm.
     */
    List<String> systemStateChoices;


    // ///////////// //
    // Constructors. //
    // ///////////// //

    /**
     * Constructor for ROSSystem
     */
    public ROSSystem(){
        systemStateChoices = Arrays.asList("Full sim", "No Roboter arm");
    }

    // ////////////////////// //
    // Read/Write properties. //
    // ////////////////////// //

    // ///////////////////// //
    // Read-only properties. //
    // ///////////////////// //

    /**
     * @return the possible System states to choose from
     */
    public List<String> getSystemStateChoices() {return systemStateChoices;}

    // //////// //
    // Methods. //
    // //////// //

    /**
     * Method that communicates with the ROS system to set the system state to the supplied one.
     * If the system state choice doesn't exist this function returns false.
     * Please use getSystemStateChoices for possible system states.
     *
     * @param ircClient the IRCCLient
     * @param systemState the system state to set the ROS system into
     * @return true if the system got set into the supplied system state
     * @throws ProtocolException thrown by sending and receiving the buffer
     * @throws IOException thrown if returned cmd does not match expected
     */
    public boolean setSystemState(IRCClient ircClient, String systemState) throws ProtocolException, IOException {

        // Checking if systemState is equal to first element in system state Choices
        if(systemState.equals(systemStateChoices.get(0))){

            // If yes set system into full simulation state
            setSystemInFullSimulationState(ircClient);

            // Return true if nothing went wrong
            return true;
        }

        // Otherwise check if system state is equal to second element in system state Choices
        else if(systemState.equals(systemStateChoices.get(1))){

            // If yes no roboter arm was selected thus set system into no roboter arm state
            setSystemNoRoboter(ircClient);

            // Return true if nothing went wrong
            return true;
        }
        else{

            // Return false if system state choice doesn't exist
            return false;
        }
    }


}
