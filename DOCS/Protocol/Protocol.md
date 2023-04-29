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
|   0x09    |   n               |   UTF-8 encoding  |   0x09            |   0               |   none            |   set search option. Data consists of <code>Unknown</code>
|   0x0A    |   n               |   UTF-8 encoding  |   0x0A            |   0               |   none            |   Chess Player move send Command <code>ChessMove</code> example: D2D4, A2A4,.....
|   0x0B    |   0               |   none            |   0x0B            |   n               |   UTF-8 encoding  |   Chess Chessengine move return Command <code>ChessMove</code> example: D2D4, A2A4,.....
|   0x0C    |   0               |   none            |   0x0C            |   0               |   none            |   Set system playing against chess engine without roboter
|   0x0D    |   0               |   none            |   0x0D            |   0               |   none            |   Set system into complete simulation with roboter simulating player and chessEngine



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
|   0xF5    |   Search option doesn't exist
|   0xF4    |   Pawn collided straight move
|   0xF3    |   Pawn collided diagonal or empty space
|   0xF2    |   Starting cell is empty
|   0xF1    |   Not that colors turn
|   0xF0    |   Move Invalid or blocked by own color
|   0xEF    |   Cannot Castle King side
|   0xEE    |   Cannot Castle Queen side
|   0xED    |   Own King in Check
|   0xEC    |   Other King in CheckMate
|   0xEB    |   Pawn not allowed there because it needs to be promotion move
|   0xEA    |   Piece to promote is not a pawn
|   0xE9    |   Pawn not moving into endPos for promotion move
|   0xE8    |   Invalid Piece to promote into
|   0xE7    |   Chess Engine created no move
|   0xE6    |   Invalid Move format
|   0xE5    |   No chess board information from camera published
|   0xE4    |   Pick Up cell is empty. Internal and real world chess board mismatch
|   0xE3    |   System already in a playing chess state machine  