/*
 *@(#) ibo1.Enum.ProtocolErrors.java 0.1 2023/04/30
 *
 */
package ibo1.Enum;

/**
 * ProtocolErrors - Enum for Protocol Errors
 * <p>
 * Enum used for Protocol Errors
 * @author Omar Ibrahim
 * @version 0.1 ( Initial development ).
 * @version 1.0 ( Initial release ).
 * @see ibo1.Protocol.User
 * @see ibo1.Protocol.ChessEngine
 *
 */
public enum ProtocolErrors {
    /**
     * For Protocol error: 0xFC
     */
    User_DOES_NOTEXIST,

    /**
     * For Protocol error that was not defined in the IRC_API for java
     */
    UNEXPECTED_RETURN_CMD,

    /**
     * For Protocol error: 0xFE
     */
    UNRECOGNIZABLE_CMD,

    /**
     * For Protocol error: 0xF9
     */
    CHESSENGINE_NOT_FOUND,

    /**
     * For Protocol error: 0xF8
     */
    CHESSENGINE_NOT_STARTED,

    /**
     * For Protocol error: 0xF7
     */
    CHESSENGINE_NOT_RUNNING,

    /**
     * For Protocol error: 0xF6
     */
    CHESSENGINE_OPTION_DOES_NOTEXIST,

    /**
     * For Protocol error: 0xF5
     */
    CHESSENGINE_SEARCH_OPTION_DOES_NOTEXIST,

    /**
     * For Protocol error: 0xF4
     */
    CHESSENGINE_PAWN_COLLIDED_STRAIGHT,

    /**
     * For Protocol error: 0xF3
     */
    CHESSENGINE_PAWN_COLLIDED_DIAGONAL_OREMPTYCELLTHERE,

    /**
     * For Protocol error: 0xF2
     */
    CHESSENGINE_STARTING_CELL_EMPTY,

    /**
     * For Protocol error: 0xF1
     */
    CHESSENGINE_NOT_THAT_COLORS_TURN,

    /**
     * For Protocol error: 0xF0
     */
    CHESSENGINE_MOVE_INVALID_OR_BLOCKED_BY_OWN_COLOR,

    /**
     * For Protocol error: 0xEF
     */
    CHESSENGINE_CANNOT_CASTLE_KING_SIDE,

    /**
     * For Protocol error: 0xEE
     */
    CHESSENGINE_CANNOT_CASTLE_QUEEN_SIDE,

    /**
     * For Protocol error: 0xEA
     */
    CHESSENGINE_PIECE_TO_PROMOTE_NOT_PAWN,

    /**
     * For Protocol error: 0xED
     */
    CHESSENGINE_OWN_KING_IN_CHECK,

    /**
     * For Protocol error: 0xEC
     */
    CHESSENGINE_OTHER_KING_IN_CHECK_MATE,

    /**
     * For Protocol error: 0xE9
     */
    CHESSENGINE_PAWN_NOT_MOVING_INTO_END_POS,

    /**
     * For Protocol error: 0xEA
     */
    CHESSENGINE_INVALID_NAME_FOR_PIECE_TO_PROMOTE_INTO,

    /**
     * For Protocol error: 0xEB
     */
    CHESSENGINE_PAWN_NOT_ALLOWED_NOT_PROMOTION_MOVE,

    /**
     * For Protocol error: 0xE7
     */
    CHESSENGINE_CREATED_NO_MOVE,

    /**
     * For Protocol error: 0xE6
     */
    CHESSENGINE_MOVE_FORMAT_INVALID,

    /**
     * For Protocol error: 0xE5
     */
    CAMERA_DOESNT_PUBLISH_INFO,

    /**
     * For Protocol error: 0xE4
     */
    INTERNAL_REAL_CHESS_BOARD_MISMATCH,

    /**
     * For Protocol error: 0xE3
     */
    SYSTEM_ALREADY_IN_CHESS_STATE



}
