<!---
 @(#) Protocol Document 0.1 2023/03/17
 
 Copyright (c) Omar Ibrahim
 All rights reserved.
 -->

# Protocol

This document consists of the definition for the protocol and the type of data that is being sent.

## Definition
The protocol consists of a client and a server. The client makes the requests whereas the server will only be used for replies.
The server should be setup on the robotic system and the client should be setup on the user interface system.



The protocol consists of following contents:
- command
    - is the command which defines what should happen
    - Size: 1byte
- sizeData
    - defines how big the data that is followed is. Thus only the needed bytes will be read and can be checked for accurate receiving of byteSize
    - Size: 4bytes
- data
    - consists of the data that is being sent in accordance to the command
    - Size: n-bytes

Further the reply will always consist of the command sent to check for the correct receiving of the command on the receiver.


## Commands

|   Command |   Size of data    |   data            |   Reply Command   |   Size of data    |   data            |   Description
|   :-----: |   :----------:    |   :--:            |   :-----------:   |   :----------:    |   :--:            |   :---------
|   0x00    |   0               |   none            |   none            |   none            |   none            |   disconnect command to tell the server to close the current connection.
|   0x01    |   0               |   none            |   0x01            |   0               |   none            |   connecting command to verify connection established. Should be first command sent
|   0x02    |   n               |   UTF-8 encoding  |   0x02            |   1byte           |   boolean         |   Logging in command. Data consists of <code>Username1F(Ascii Unit Seperator)Password</code>, returns true/false for isAdmin
|   0x03    |   n               |   UTF-8 encoding  |   0x03            |   0               |   none            |   creating User in command. Data consists of <code>Username1F(Ascii Unit Seperator)Password</code>, returns true/false for isAdmin
|   0x04    |   0               |   none            |   0x04            |   n               |   UTF-8 encoding  |   getPossible Chess engines. Return data consists of each <code>chessEngine11F(Ascii Unit Seperator)chessEngine1F(Ascii Unit Seperator)chessEngine....</code>
|   0x05    |   n               |   UTF-8 encoding  |   0x05            |   0               |   none            |   start chess Engine. Data consists of <code>chessEngineName</code>
|   0x06    |   0               |   none            |   0x06            |   0               |   none            |   stop chess Engine.
|   0x07    |   0               |   none            |   0x07            |   n               |   UTF-8 encoding  |   get chess engine options. Data consists of <code>Unknown</code>
|   0x08    |   n               |   UTF-8 encoding  |   0x08            |   0               |   none            |   set chess engine option. Data consists of <code>Unknown</code>
|   0x09    |   n               |   UTF-8 encoding  |   0x09            |   0               |   none            |   Chess Player move send Command <code>ChessMove</code> example: D2D4, A2A4,.....
|   0x0A    |   0               |   none            |   0x0A            |   n               |   UTF-8 encoding  |   Chess Chessengine move return Command <code>ChessMove</code> example: D2D4, A2A4,.....
|   0x0B    |   n               |   UTF-8 encoding  |   0x0B            |   0               |   none            |   set search option. Data consists of <code>Unknown</code>


## Error Codes
|   Value   |   Description
|   :---:   |   :----------
|   0xFF    |   Reading bytes error
|   0xFE    |   Command doesn't exist
|   0xFD    |   Client not yet connected.
|   0xFC    |   User does not exist.
|   0xFB    |   User name already exists.
|   0xFA    |   User couldn't be created.
|   0xF9    |   Chess Engine could not be found.
|   0xF8    |   Chess Engine could not be started.
|   0xF7    |   No instance of chess engine running.
|   0xF6    |   Chess Engine option doesnt exist.
|   0xF5    |   Pawn collided straight move
|   0xF4    |   Pawn collided diagonal or empty space
|   0xF3    |   Starting cell is empty
|   0xF2    |   Not that colors turn
|   0xF1    |   Move Invalid or blocked by own color
|   0xF0    |   Cannot Castle King side
|   0xEF    |   Cannot Castle Queen side
|   0xEE    |   Move format invalid
|   0xEE    |   Search option doesn't exist