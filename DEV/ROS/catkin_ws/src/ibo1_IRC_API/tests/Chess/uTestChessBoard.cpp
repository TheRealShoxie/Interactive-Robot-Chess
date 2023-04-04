#include <ros/ros.h>

#include "ibo1_IRC_API/Chess/ChessBoard.h"

#include <gtest/gtest.h>

#include "iostream"

using namespace std;


//Testing chessBoard creation standardPos
TEST(ChessBoard, creationStandardPos){
    ChessBoard chessBoard = ChessBoard();
    string fenPos = "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1";

    ASSERT_EQ(chessBoard.toFENString(), fenPos);
}

//Testing chessBoard creation with fenString
TEST(ChessBoard, creationWithFenString){
    string fenPos = "r1bqkbnr/ppp2ppp/2n5/3pp3/2P5/3BPN2/PP1P1PPP/RNBQK2R w KQkq - 0 5";
    ChessBoard chessBoard = ChessBoard(fenPos);

    ASSERT_EQ(chessBoard.toFENString(), fenPos);
}

//Testing chessBoard creation with fenString2
TEST(ChessBoard, creationWithFenString2){
    string fenPos = "r3kbnr/ppp2ppp/2n5/3p2q1/P1P5/1P2P1P1/5P1P/1bB2RK1 w kq - 0 13";
    ChessBoard chessBoard = ChessBoard(fenPos);

    ASSERT_EQ(chessBoard.toFENString(), fenPos);
}

//Testing chessBoard creation with fenString3
TEST(ChessBoard, creationWithFenString3){
    string fenPos = "r3kbnr/ppp2ppp/2n5/3p2q1/P1P5/1P1bPPP1/7P/2B2RK1 w kq - 1 14";
    ChessBoard chessBoard = ChessBoard(fenPos);

    ASSERT_EQ(chessBoard.toFENString(), fenPos);
}

//Testing chessBoard creation with fenString4
TEST(ChessBoard, creationWithFenString4){
    string fenPos = "r3kbnr/ppp2ppp/8/4n1q1/P1p5/1P1bPPP1/7P/2BR2K1 w kq - 2 16";
    ChessBoard chessBoard = ChessBoard(fenPos);

    ASSERT_EQ(chessBoard.toFENString(), fenPos);
}

//Testing chessBoard move from standard Position valid pawn move
TEST(ChessBoard, MoveValidFromStartPosValidPawnMove){
    ChessBoard chessBoard = ChessBoard();
    string movement = "A2A4";
    int expectedReturn = 0;

    int returned = chessBoard.move(movement);

    ASSERT_EQ(returned, expectedReturn);
}

//Testing chessBoard move from standard Position invalid pawn move diagonal
TEST(ChessBoard, MoveValidFromStartPosInValidPawnMoveDiagonal){
    ChessBoard chessBoard = ChessBoard();
    string movement = "A2B3";
    int expectedReturn = -2;

    int returned = chessBoard.move(movement);

    ASSERT_EQ(returned, expectedReturn);
}

//Testing chessBoard move from standard Position invalid queen move piece in the way
TEST(ChessBoard, MoveValidFromStartPosInValidQueenMove){
    ChessBoard chessBoard = ChessBoard();
    string movement = "D1D2";
    int expectedReturn = -5;

    int returned = chessBoard.move(movement);

    ASSERT_EQ(returned, expectedReturn);
}

//Testing chessBoard move from standard Position invalid knight move
TEST(ChessBoard, MoveValidFromStartPosInValidKnightMove){
    ChessBoard chessBoard = ChessBoard();
    string movement = "B1B2";
    int expectedReturn = -5;

    int returned = chessBoard.move(movement);

    ASSERT_EQ(returned, expectedReturn);
}

//Testing chessBoard move from standard Position valid knight move
TEST(ChessBoard, MoveValidFromStartPosValidKnightMove){
    ChessBoard chessBoard = ChessBoard();
    string movement = "B1C3";
    int expectedReturn = 0;

    int returned = chessBoard.move(movement);
    ASSERT_EQ(returned, expectedReturn);
}


//Testing chessBoard move from FenPosition invalid pawn move double black in the way
TEST(ChessBoard, MoveValidFromFenPosInValidPawnMoveDouble){
    string fenPos = "rnbqkbnr/p1pppppp/8/P7/8/1p6/1PPPPPPP/RNBQKBNR w KQkq - 0 4";
    ChessBoard chessBoard = ChessBoard(fenPos);

    string movement = "B2B4";
    int expectedReturn = -1;

    int returned = chessBoard.move(movement);

    ASSERT_EQ(returned, expectedReturn);
}

//Testing chessBoard move from FenPosition invalid pawn move double black in the way
TEST(ChessBoard, MoveValidFromStartPosInValidPawnMoveQuadruple){
    ChessBoard chessBoard = ChessBoard();

    string movement = "A2A6";
    int expectedReturn = -5;

    int returned = chessBoard.move(movement);

    ASSERT_EQ(returned, expectedReturn);
}

//Testing chessBoard move from FenPosition invalid pawn move single black in the way
TEST(ChessBoard, MoveValidFromFenPosInValidPawnMoveSingle){
    string fenPos = "rnbqkbnr/p1pppppp/8/P7/8/1p6/1PPPPPPP/RNBQKBNR w KQkq - 0 4";
    ChessBoard chessBoard = ChessBoard(fenPos);

    string movement = "B2B3";
    int expectedReturn = -1;

    int returned = chessBoard.move(movement);

    ASSERT_EQ(returned, expectedReturn);
}

//Testing chessBoard move from FenPosition valid pawn move diagonal
TEST(ChessBoard, MoveValidFromFenPosValidPawnMoveDiagonal){
    string fenPos = "rnbqkbnr/p1pppppp/8/P7/8/1p6/1PPPPPPP/RNBQKBNR w KQkq - 0 4";
    ChessBoard chessBoard = ChessBoard(fenPos);

    string movement = "C2B3";
    int expectedReturn = 0;

    int returned = chessBoard.move(movement);

    ASSERT_EQ(returned, expectedReturn);
}

//Testing chessBoard move from FenPosition invalid pawn move diagonal empty cell
TEST(ChessBoard, MoveValidFromFenPosInValidPawnMoveDiagonalEmpty){
    string fenPos = "rnbqkbnr/p1pppppp/8/P7/8/1p6/1PPPPPPP/RNBQKBNR w KQkq - 0 4";
    ChessBoard chessBoard = ChessBoard(fenPos);

    string movement = "C2D3";
    int expectedReturn = -2;

    int returned = chessBoard.move(movement);

    ASSERT_EQ(returned, expectedReturn);
}

//Testing chessBoard move from FenPosition invalid pawn move diagonal same piece there
TEST(ChessBoard, MoveValidFromFenPosInValidPawnMoveDiagonalSameColorPiece){
    string fenPos = "rnbqkbnr/p1pp1ppp/4p3/P7/8/1p1P4/1PP1PPPP/RNBQKBNR w KQkq - 0 5";
    ChessBoard chessBoard = ChessBoard(fenPos);

    string movement = "C2D3";
    int expectedReturn = -5;

    int returned = chessBoard.move(movement);

    ASSERT_EQ(returned, expectedReturn);
}

//Testing chessBoard move from FenPosition invalid bishop move diagonal same piece there
TEST(ChessBoard, MoveValidFromFenPosInValidBishopMoveDiagonalSameColorPiece){
    string fenPos = "rnbqkbnr/p1pp1ppp/4p3/P7/8/1p1P4/1PP1PPPP/RNBQKBNR w KQkq - 0 5";
    ChessBoard chessBoard = ChessBoard(fenPos);

    string movement = "C1A3";
    int expectedReturn = -5;

    int returned = chessBoard.move(movement);

    ASSERT_EQ(returned, expectedReturn);
}

//Testing chessBoard move from FenPosition valid bishop move diagonal same piece there
TEST(ChessBoard, MoveValidFromFenPosValidBishopMoveDiagonal){
    string fenPos = "rnbqkbnr/p1pp1ppp/4p3/P7/8/1p1P4/1PP1PPPP/RNBQKBNR w KQkq - 0 5";
    ChessBoard chessBoard = ChessBoard(fenPos);

    string movement = "C1F4";
    int expectedReturn = 0;

    int returned = chessBoard.move(movement);

    ASSERT_EQ(returned, expectedReturn);
}

//Testing chessBoard move from FenPosition valid bishop diagonal taking knight
TEST(ChessBoard, MoveValidFromFenPosValidBishopMoveDiagonalTakeKnight){
    string fenPos = "rn1qkbnr/p1pp1pp1/2P1p1b1/P6p/3P4/1p4P1/1P2PP1P/RNBQKBNR b KQkq - 0 10";
    ChessBoard chessBoard = ChessBoard(fenPos);

    string movement = "G6B1";
    int expectedReturn = 0;

    int returned = chessBoard.move(movement);

    ASSERT_EQ(returned, expectedReturn);
}

//Testing chessBoard move from FenPosition invalid rook move up
TEST(ChessBoard, MoveValidFromFenPosInValidRookMoveUp){
    string fenPos = "rnbqkbnr/p1pp1ppp/4p3/P7/8/1p1P4/1PP1PPPP/RNBQKBNR w KQkq - 0 5";
    ChessBoard chessBoard = ChessBoard(fenPos);

    string movement = "H1H6";
    int expectedReturn = -5;

    int returned = chessBoard.move(movement);

    ASSERT_EQ(returned, expectedReturn);
}

//Testing chessBoard move from FenPosition invalid rook2 move up
TEST(ChessBoard, MoveValidFromFenPosInValidRookMoveUp2){
    string fenPos = "rnbqkbnr/p1pp1ppp/4p3/P7/8/1p1P4/1PP1PPPP/RNBQKBNR w KQkq - 0 5";
    ChessBoard chessBoard = ChessBoard(fenPos);

    string movement = "A1A5";
    int expectedReturn = -5;

    int returned = chessBoard.move(movement);

    ASSERT_EQ(returned, expectedReturn);
}

//Testing chessBoard move from FenPosition valid rook move up
TEST(ChessBoard, MoveValidFromFenPosValidRookMoveUp){
    string fenPos = "rnbqkbnr/p2ppppp/2p5/P7/8/1p1P4/1PP1PPPP/RNBQKBNR w KQkq - 0 5";
    ChessBoard chessBoard = ChessBoard(fenPos);

    string movement = "A1A3";
    int expectedReturn = 0;

    int returned = chessBoard.move(movement);

    ASSERT_EQ(returned, expectedReturn);
}

//Testing chessBoard move from FenPosition to Moved FenPosition
TEST(ChessBoard, MoveHasBeenMadeToNewFenPosition){
    string fenPosStart = "rnbqkbnr/p2ppppp/2p5/P7/8/1p1P4/1PP1PPPP/RNBQKBNR w KQkq - 0 5";
    string fenPosExpected = "rnbqkbnr/p2ppppp/2p5/P7/8/Rp1P4/1PP1PPPP/1NBQKBNR b Kkq - 1 5";
    ChessBoard chessBoard = ChessBoard(fenPosStart);

    string movement = "A1A3";

    chessBoard.move(movement);
    string returnedFenPos = chessBoard.toFENString();

    ASSERT_EQ(returnedFenPos, fenPosExpected);
}

//Testing chessBoard move from FenPosition to Moved FenPosition
TEST(ChessBoard, MoveHasBeenMadeToNewFenPosition2){
    string fenPosStart = "rnbqkbnr/p2ppppp/2p5/P7/8/1p1P4/1PP1PPPP/RNBQKBNR w KQkq - 0 5";
    string fenPosExpected = "rnbqkbnr/p2ppppp/2p5/P7/5B2/1p1P4/1PP1PPPP/RN1QKBNR b KQkq - 1 5";
    ChessBoard chessBoard = ChessBoard(fenPosStart);

    string movement = "C1F4";

    chessBoard.move(movement);
    string returnedFenPos = chessBoard.toFENString();

    ASSERT_EQ(returnedFenPos, fenPosExpected);
}


//Testing chessBoard castle white king side from fenPosition invalid
TEST(ChessBoard, CastleWhiteKingSideFromFenPositionInvalid){
    string fenPosStart = "rnbqkbnr/ppp1p1pp/3p4/5p2/4P3/5N2/PPPP1PPP/RNBQKB1R w KQkq f6 0 3";
    int expectedReturn = -5;
    ChessBoard chessBoard = ChessBoard(fenPosStart);

    string movement = "e1g1";

    int returned = chessBoard.move(movement);

    ASSERT_EQ(returned, expectedReturn);
}

//Testing chessBoard castle white king side from fenPosition valid
TEST(ChessBoard, CastleWhiteKingSideFromFenPositionValid){
    string fenPosStart = "rnbqkbnr/ppp3pp/3pp3/5p2/4P3/3B1N2/PPPP1PPP/RNBQK2R w KQkq - 0 4";
    int expectedReturn = 0;
    ChessBoard chessBoard = ChessBoard(fenPosStart);

    string movement = "e1g1";

    int returned = chessBoard.move(movement);

    ASSERT_EQ(returned, expectedReturn);
}

//Testing chessBoard castle white king side from fenPosition invalid as no permission
TEST(ChessBoard, CastleWhiteKingSideFromFenPositionInValidMissingCastleRights){
    string fenPosStart = "rnbqkbnr/ppp4p/3pp3/5pp1/4P3/3B1N2/PPPP1PPP/RNBQK2R w kq - 0 6";
    int expectedReturn = -6;
    ChessBoard chessBoard = ChessBoard(fenPosStart);

    string movement = "E1G1";

    int returned = chessBoard.move(movement);

    ASSERT_EQ(returned, expectedReturn);
}

//Testing chessBoard promotion from fenpos allowed into a queen
TEST(ChessBoard, PawnPromotionFromFenPositionValidIntoQueen){
    string fenPosStart = "1k2rbnr/p2Ppppp/Pp1p4/2p2b2/8/4P3/2PP1PPP/RNBQKBNR w KQ - 1 11";
    int expectedReturn = 0;
    ChessBoard chessBoard = ChessBoard(fenPosStart);

    string movement = "d7d8q";

    int returned = chessBoard.move(movement);

    ASSERT_EQ(returned, expectedReturn);
}

//Testing chessBoard promotion from fenpos not allowed blocked by piece
TEST(ChessBoard, PawnPromotionFromFenPositionInValidBlockedByPiece){
    string fenPosStart = "1k1r1bnr/p2Ppp1p/Pp1p2p1/2p2b2/8/4P3/2PP1PPP/RNBQKBNR w KQ - 0 11";
    int expectedReturn = -1;
    ChessBoard chessBoard = ChessBoard(fenPosStart);

    string movement = "d7d8q";

    int returned = chessBoard.move(movement);


    ASSERT_EQ(returned, expectedReturn);
}

//Testing chessBoard promotion from fenpos allowed into Knight
TEST(ChessBoard, PawnPromotionFromFenPositionValidIntoKnight){
    string fenPosStart = "1k2rbnr/p2Ppppp/Pp1p4/2p2b2/8/4P3/2PP1PPP/RNBQKBNR w KQ - 1 11";
    int expectedReturn = 0;
    ChessBoard chessBoard = ChessBoard(fenPosStart);

    string movement = "d7d8n";

    int returned = chessBoard.move(movement);


    ASSERT_EQ(returned, expectedReturn);
}

//Testing chessBoard promotion from fenpos allowed into Rook
TEST(ChessBoard, PawnPromotionFromFenPositionValidIntoRook){
    string fenPosStart = "1k2rbnr/p2Ppppp/Pp1p4/2p2b2/8/4P3/2PP1PPP/RNBQKBNR w KQ - 1 11";
    int expectedReturn = 0;
    ChessBoard chessBoard = ChessBoard(fenPosStart);

    string movement = "d7d8r";

    int returned = chessBoard.move(movement);

    ASSERT_EQ(returned, expectedReturn);
}

//Testing chessBoard promotion from fenpos allowed into Bishop
TEST(ChessBoard, PawnPromotionFromFenPositionValidIntoBishop){
    string fenPosStart = "1k2rbnr/p2Ppppp/Pp1p4/2p2b2/8/4P3/2PP1PPP/RNBQKBNR w KQ - 1 11";
    int expectedReturn = 0;
    ChessBoard chessBoard = ChessBoard(fenPosStart);

    string movement = "d7d8b";

    int returned = chessBoard.move(movement);

    ASSERT_EQ(returned, expectedReturn);
}

//Testing chessBoard promotion from fenpos not allowed invalid Piece
TEST(ChessBoard, PawnPromotionFromFenPositionInValidIntoNotAllowedPiece){
    string fenPosStart = "1k2rbnr/p2Ppppp/Pp1p4/2p2b2/8/4P3/2PP1PPP/RNBQKBNR w KQ - 1 11";
    int expectedReturn = -12;
    ChessBoard chessBoard = ChessBoard(fenPosStart);

    string movement = "d7d8k";

    int returned = chessBoard.move(movement);


    ASSERT_EQ(returned, expectedReturn);
}

//Testing chessBoard promotion from fenpos not allowed not end Position
TEST(ChessBoard, PawnPromotionFromFenPositionInValidNotEndPosition){
    string fenPosStart = "1k2rbnr/p2Ppppp/Pp1p4/2p2b2/8/4P3/2PP1PPP/RNBQKBNR w KQ - 1 11";
    int expectedReturn = -11;
    ChessBoard chessBoard = ChessBoard(fenPosStart);

    string movement = "c2c3n";

    int returned = chessBoard.move(movement);

    ASSERT_EQ(returned, expectedReturn);
}

//Testing chessBoard promotion from fenpos not allowed not a pawn
TEST(ChessBoard, PromotionFromFenPositionInvalidNotPawnPiece){
    string fenPosStart = "1k2rbnr/p1RPp1pp/Pp3p2/5b2/3p4/4P3/2PP1PPP/1NBQKBNR w K - 0 14";
    int expectedReturn = -10;
    ChessBoard chessBoard = ChessBoard(fenPosStart);

    string movement = "c7c8n";

    int returned = chessBoard.move(movement);

    ASSERT_EQ(returned, expectedReturn);
}

//Testing chessBoard promotion from fenpos allowed black pawn into a queen
TEST(ChessBoard, BlackPawnPromotionFromFenPositionValidIntoQueen){
    string fenPosStart = "1k2rbnr/p1RPp1pp/Pp3p2/8/4P1b1/B1N2P2/2pP2PP/3QKBNR b K - 3 17";
    int expectedReturn = 0;
    ChessBoard chessBoard = ChessBoard(fenPosStart);

    string movement = "c2c1q";

    int returned = chessBoard.move(movement);

    ASSERT_EQ(returned, expectedReturn);
}

//Testing chessBoard promotion from fenpos allowed black pawn into a queen while taking
TEST(ChessBoard, BlackPawnPromotionFromFenPositionTakeValidIntoQueen){
    string fenPosStart = "1k2rbnr/p1RPp1pp/Pp3p2/8/4P1b1/B1N2P2/2pP2PP/3QKBNR b K - 3 17";
    int expectedReturn = 0;
    ChessBoard chessBoard = ChessBoard(fenPosStart);

    string movement = "c2d1q";

    int returned = chessBoard.move(movement);

    ASSERT_EQ(returned, expectedReturn);
}

//Testing chessBoard promotion from fenpos not black allowed blocked by piece
TEST(ChessBoard, BlackPawnPromotionFromFenPositionInValidBlockedByPiece){
    string fenPosStart = "1k2rbnr/p1RPp1pp/Pp3p2/8/4PPb1/2N5/2pP2PP/2BQKBNR b K - 0 17";
    int expectedReturn = -1;
    ChessBoard chessBoard = ChessBoard(fenPosStart);

    string movement = "c2c1q";

    int returned = chessBoard.move(movement);


    ASSERT_EQ(returned, expectedReturn);
}

//Testing chessBoard promotion from fenpos black pawn allowed into Knight
TEST(ChessBoard, BlackPawnPromotionFromFenPositionValidIntoKnight){
    string fenPosStart = "1k2rbnr/p1RPp1pp/Pp3p2/8/4P1b1/B1N2P2/2pP2PP/3QKBNR b K - 3 17";
    int expectedReturn = 0;
    ChessBoard chessBoard = ChessBoard(fenPosStart);

    string movement = "c2c1n";

    int returned = chessBoard.move(movement);


    ASSERT_EQ(returned, expectedReturn);
}

//Testing chessBoard promotion from fenpos black pawn allowed into Rook
TEST(ChessBoard, BlackPawnPromotionFromFenPositionValidIntoRook){
    string fenPosStart = "1k2rbnr/p1RPp1pp/Pp3p2/8/4P1b1/B1N2P2/2pP2PP/3QKBNR b K - 3 17";
    int expectedReturn = 0;
    ChessBoard chessBoard = ChessBoard(fenPosStart);

    string movement = "c2c1r";

    int returned = chessBoard.move(movement);

    ASSERT_EQ(returned, expectedReturn);
}

//Testing chessBoard promotion from fenpos black pawn allowed into Bishop
TEST(ChessBoard, BlackPawnPromotionFromFenPositionValidIntoBishop){
    string fenPosStart = "1k2rbnr/p1RPp1pp/Pp3p2/8/4P1b1/B1N2P2/2pP2PP/3QKBNR b K - 3 17";
    int expectedReturn = 0;
    ChessBoard chessBoard = ChessBoard(fenPosStart);

    string movement = "c2c1b";

    int returned = chessBoard.move(movement);

    ASSERT_EQ(returned, expectedReturn);
}

//Testing chessBoard promotion from fenpos black pawn not allowed invalid Piece
TEST(ChessBoard, BlackPawnPromotionFromFenPositionInValidIntoNotAllowedPiece){
    string fenPosStart = "1k2rbnr/p1RPp1pp/Pp3p2/8/4P1b1/B1N2P2/2pP2PP/3QKBNR b K - 3 17";
    int expectedReturn = -12;
    ChessBoard chessBoard = ChessBoard(fenPosStart);

    string movement = "c2c1k";

    int returned = chessBoard.move(movement);


    ASSERT_EQ(returned, expectedReturn);
}

//Testing chessBoard promotion from fenpos black pawn not allowed not end Position
TEST(ChessBoard, BlackPawnPromotionFromFenPositionInValidNotEndPosition){
    string fenPosStart = "1k2rbnr/p1RPp1pp/Pp3p2/8/4P1b1/B1N2P2/2pP2PP/3QKBNR b K - 3 17";
    int expectedReturn = -11;
    ChessBoard chessBoard = ChessBoard(fenPosStart);

    string movement = "b6b5n";

    int returned = chessBoard.move(movement);

    ASSERT_EQ(returned, expectedReturn);
}

//Testing chessBoard promotion from fenpos black pawn not allowed not a pawn
TEST(ChessBoard, BlackPromotionFromFenPositionInvalidNotPawnPiece){
    string fenPosStart = "1k2rbnr/p1RPp1pp/Pp3p2/8/4P1b1/B1N2P2/2pP2PP/3QKBNR b K - 3 17";
    int expectedReturn = -10;
    ChessBoard chessBoard = ChessBoard(fenPosStart);

    string movement = "g4h3n";

    int returned = chessBoard.move(movement);

    ASSERT_EQ(returned, expectedReturn);
}

//Testing chessBoard white pawn move not allowed not a promotion move
TEST(ChessBoard, PawnMoveFromFenPositionInvalidNotPromotionMove){
    string fenPosStart = "1k2rbnr/p1RPp2p/Pp3pp1/8/4PP2/2N4b/1BpP2PP/3QKBNR w K - 0 19";
    int expectedReturn = -9;
    ChessBoard chessBoard = ChessBoard(fenPosStart);

    string movement = "d7d8";

    int returned = chessBoard.move(movement);

    ASSERT_EQ(returned, expectedReturn);
}


//Testing chessBoard white pawn move not allowed not a promotion move
TEST(ChessBoard, WhiteKingCheckStartPos){
    string fenPosStart = "rnbqk1nr/pp3ppp/2p1p3/Q2p2B1/8/N1PPb1P1/PP2PP1P/R3KBNR w KQkq - 0 8";
    int expectedReturn = 0;
    ChessBoard chessBoard = ChessBoard(fenPosStart);

    int startPos = 60;
    int endPos = 59;
    
    int returned = chessBoard.kingCheck(startPos, endPos, true);

    ASSERT_EQ(returned, expectedReturn);
}


int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "testChessBoard");
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();

}
