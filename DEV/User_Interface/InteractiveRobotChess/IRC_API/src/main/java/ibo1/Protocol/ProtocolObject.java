/*
 *@(#) ibo1.Protocol.ProtocolObject.java 0.1 2023/03/17
 *
 */
package ibo1.Protocol;

import java.nio.ByteBuffer;

/**
 * ProtocolObject - Represents the core structure of the protocol
 * @author Omar Ibrahim
 * @version 0.1 ( Initial development ).
 * @version 1.0 ( Initial release ).
 * @see ByteBuffer
 * @see User
 * @see ibo1.Client.IRCClient
 */
public class ProtocolObject {

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

    /**
     * Represents the cmd byte of the protocol
     */
    private byte cmdByte;


    /**
     * Represents the size of the data in the protocol data section
     */
    private int dataSize;


    /**
     * Represents the data that is being sent
     */
    private byte[] data;


    // ///////////// //
    // Constructors. //
    // ///////////// //

    /**
     * Default constructor for ProtocolObject.
     * Sets instance to default values.
     */
    public ProtocolObject(){
        cmdByte = 0x00;
        dataSize = 0;
        data = new byte[0];
    }

    // ////////////////////// //
    // Read/Write properties. //
    // ////////////////////// //


    /**
     *
     * @return Command Byte of the protocol
     */
    public byte getCmdByte() {
        return cmdByte;
    }

    /**
     *
     * @return The size of the data as an int
     */
    public int getDataSize() {
        return dataSize;
    }

    /**
     *
     * @return Data that is followed.
     */
    public byte[] getData() {
        return data;
    }

    /**
     *
     * @param cmdByte Byte value of the command byte
     */
    public void setCmdByte(byte cmdByte) {
        this.cmdByte = cmdByte;
    }

    /**
     *
     * @param dataSize dataSize as an Integer
     */
    public void setDataSize(int dataSize){
        this.dataSize = dataSize;
    }

    /**
     *
     * @param dataSize Byte array of the dataSize
     */
    public void setDataSize(byte[] dataSize) {
        this.dataSize = ByteBuffer.wrap(dataSize).getInt();
    }


    /**
     * Used to set the Data. If length of parsed data not greater 0 then empty array will be set as data.
     *
     * @param data Data that has been received/sent
     */
    public void setData(byte[] data) {
        if(data.length != 0){
            this.data = data;
            this.dataSize = data.length;
        } else {
            this.data = new byte[0];
            this.dataSize = 0;
        }
    }

    // ///////////////////// //
    // Read-only properties. //
    // ///////////////////// //

    // //////// //
    // Methods. //
    // //////// //

    /**
     *
     * @return The size of the data as a byte array
     */
    private byte[] getDataSizeAsByteArray(){
        return ByteBuffer.allocate(4).putInt(dataSize).array();
    }

    /**
     *
     * @return - Protocol Object as a byte array.
     */
    public byte[] toByteArray(){
        return ByteBuffer.allocate(1 + 4 + data.length)
                .put(cmdByte)
                .put(getDataSizeAsByteArray())
                .put(getData())
                .array();
    }

    /**
     * Creates a proper to string for the ProtocolObject.
     *
     * @return String representation of ProtocolObject
     */
    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder();
        sb.append("Command Byte: ");
        sb.append(String.format("%02X", cmdByte));
        sb.append("\nData Size: ");
        sb.append(dataSize);
        sb.append("\nData: ");
        for(byte b : data){
            sb.append("[0x" + String.format("%02X", b) + "]\n");
        }

        return sb.toString();
    }
}