#include "ibo1_irc_api/Chess/ChessBoard.h"
#include "iostream"

    // ///////////// //
    // Constructors. //
    // ///////////// //
    ChessBoard::ChessBoard(){
        castleRights[0][0] = true;
        castleRights[0][1] = true;
        castleRights[1][0] = true;
        castleRights[1][1] = true;

        whiteTurn = true;
        previousMoveWasPawn = false;
        whiteKingInCheck = false;
        blackKingInCheck = false;
        lastMoveCastle = false;

        wholeMoves = 1;
        halfMoves = 0;

        for(int i = 0; i < 64; i++){
            chessBoard.push_back(Cell(i));
        }

        // Setting black Pieces
        chessBoard.at(0).setChessPiece(new Rook(false));
        chessBoard.at(1).setChessPiece(new Knight(false));
        chessBoard.at(2).setChessPiece(new Bishop(false));
        chessBoard.at(3).setChessPiece(new Queen(false));
        chessBoard.at(4).setChessPiece(new King(false));
        chessBoard.at(5).setChessPiece(new Bishop(false));
        chessBoard.at(6).setChessPiece(new Knight(false));
        chessBoard.at(7).setChessPiece(new Rook(false));

        //Pawns black
        for(int i = 8; i < 16; i++){
            chessBoard.at(i).setChessPiece(new Pawn(false));
        }

        // Setting white Pieces
        chessBoard.at(63).setChessPiece(new Rook(true));
        chessBoard.at(62).setChessPiece(new Knight(true));
        chessBoard.at(61).setChessPiece(new Bishop(true));
        chessBoard.at(60).setChessPiece(new King(true));
        chessBoard.at(59).setChessPiece(new Queen(true));
        chessBoard.at(58).setChessPiece(new Bishop(true));
        chessBoard.at(57).setChessPiece(new Knight(true));
        chessBoard.at(56).setChessPiece(new Rook(true));

        //Pawns white
        for(int i = 55; i > 47; i--){
            chessBoard.at(i).setChessPiece(new Pawn(true));
        }
    }

    ChessBoard::ChessBoard(string const &FenPosition){
        
        for(int i = 0; i < 64; i++){
            chessBoard.push_back(Cell(i));
        }
        previousMoveWasPawn = false;

        //Convert fenString to chessBoard
        fromFENString(FenPosition);
    }

    // ////////////// //
    // Class methods. //
    // ////////////// //

    void ChessBoard::fromFENString(string const &fenPos){

        int whiteSpaceCounter = 0;
        int chessBoardPosition = 0;

        whiteKingInCheck = false;
        blackKingInCheck = false;
        lastMoveCastle = false;

        castleRights[0][0] = false;
        castleRights[0][1] = false;
        castleRights[1][0] = false;
        castleRights[1][1] = false;


        // Iterating through the whole FenString
        for(int pos = 0; pos < fenPos.length(); pos++){

            // Checking if we still have not found a white space. Meaning we are still assigning the pieces
            if(whiteSpaceCounter == 0){
                
                //----------White Pieces----------
                // Checking if it is a white rook
                if(fenPos[pos] == 'R'){
                    chessBoard.at(chessBoardPosition).setChessPiece(new Rook(true));
                    chessBoardPosition ++;
                }
                // Checking if it is a white Knight
                else if(fenPos[pos] == 'N'){
                    chessBoard.at(chessBoardPosition).setChessPiece(new Knight(true));
                    chessBoardPosition ++;
                }
                // Checking if it is a white Bishop
                else if(fenPos[pos] == 'B'){
                    chessBoard.at(chessBoardPosition).setChessPiece(new Bishop(true));
                    chessBoardPosition ++;
                }
                // Checking if it is a white Queen
                else if(fenPos[pos] == 'Q'){
                    chessBoard.at(chessBoardPosition).setChessPiece(new Queen(true));
                    chessBoardPosition ++;
                }
                // Checking if it is a white King
                else if(fenPos[pos] == 'K'){
                    chessBoard.at(chessBoardPosition).setChessPiece(new King(true));
                    chessBoardPosition ++;
                }
                // Checking if it is a white Pawn
                else if(fenPos[pos] == 'P'){
                    chessBoard.at(chessBoardPosition).setChessPiece(new Pawn(true));
                    chessBoardPosition ++;
                }

                //----------Black Pieces----------

                // Checking if it is a white rook
                else if(fenPos[pos] == 'r'){
                    chessBoard.at(chessBoardPosition).setChessPiece(new Rook(false));
                    chessBoardPosition ++;
                }
                // Checking if it is a white Knight
                else if(fenPos[pos] == 'n'){
                    chessBoard.at(chessBoardPosition).setChessPiece(new Knight(false));
                    chessBoardPosition ++;
                }
                // Checking if it is a white Bishop
                else if(fenPos[pos] == 'b'){
                    chessBoard.at(chessBoardPosition).setChessPiece(new Bishop(false));
                    chessBoardPosition ++;
                }
                // Checking if it is a white Queen
                else if(fenPos[pos] == 'q'){
                    chessBoard.at(chessBoardPosition).setChessPiece(new Queen(false));
                    chessBoardPosition ++;
                }
                // Checking if it is a white King
                else if(fenPos[pos] == 'k'){
                    chessBoard.at(chessBoardPosition).setChessPiece(new King(false));
                    chessBoardPosition ++;
                }
                // Checking if it is a white Pawn
                else if(fenPos[pos] == 'p'){
                    chessBoard.at(chessBoardPosition).setChessPiece(new Pawn(false));
                    chessBoardPosition ++;
                }

                //----------Checking for empty cells----------
                else if(0 < (fenPos[pos] - '0') && (fenPos[pos] - '0') <= 8){
                    int numberOfEmptySpaces = fenPos[pos] - '0';bool cellIsOccupied;
                    for(int emptySpace = 0; emptySpace < numberOfEmptySpaces; emptySpace++){
                        chessBoard.at(chessBoardPosition).setChessPiece(NULL);
                        chessBoardPosition ++;
                    }

                }
                // Checking if we got a white space
                else if(fenPos[pos] == ' '){
                    whiteSpaceCounter ++;
                }
                //Otherwise we got an error - unexpected
                else{
                    // Error unexpected values for Piece positions
                }
            }
            // Starting to check color turn
            else if (whiteSpaceCounter == 1){
                // Checking if its white turn
                if(fenPos[pos] == 'w'){
                    whiteTurn = true;
                }
                // Checking if its black turn
                else if(fenPos[pos] == 'b'){
                    whiteTurn = false;
                }
                // Checking if we got a white space
                else if(fenPos[pos] == ' '){
                    whiteSpaceCounter ++;
                }
                // Otherwise we got an error - unexpected
                else{
                    // Error for w or b expected
                }
            }
            //Checking for castle rights
            else if (whiteSpaceCounter == 2){
                // Checking if white has Kingside castle rights
                if(fenPos[pos] == 'K'){
                    castleRights[0][0] = true;
                }
                // Checking if white has Queenside castle rights
                else if(fenPos[pos] == 'Q'){
                    castleRights[0][1] = true;
                }
                // Checking if black has Kingside castle rights
                else if(fenPos[pos] == 'k'){
                    castleRights[1][0] = true;
                }
                // Checking if black has Queenside castle rights
                else if(fenPos[pos] == 'q'){
                    castleRights[1][1] = true;
                }
                // Checking if we have no castle Rights at all
                else if(fenPos[pos] == '-'){
                    // Do nothing as we already set all castle Rights to false
                }
                // EChecking if we got a white space
                else if(fenPos[pos] == ' '){
                    whiteSpaceCounter ++;
                }
                // Otherwise we got an error - unexpected
                else{
                    // Error for castle rights
                }
            }
            // Checking for Enpassant
            else if(whiteSpaceCounter == 3){
                // Checking if we have got a white space
                if(fenPos[pos] == ' '){
                    whiteSpaceCounter ++;
                }
                // Otherwise we got an error - unexpected
                else{
                    // No error as there is no implementation for en passant
                }
            }
            // Checking for half move
            else if(whiteSpaceCounter == 4){
                if((fenPos[pos] - '0') >= 0 && (fenPos[pos] - '0') <= 9){

                    //Checking if we have a two digit number
                    if((fenPos[pos+1] - '0') > 0 && (fenPos[pos+1] - '0') <= 9){
                        halfMoves = (fenPos[pos] - '0')*10 + (fenPos[pos+1] - '0');
                        pos++;

                    }
                    // Otherwise we do one digit
                    else{
                        halfMoves = fenPos[pos] - '0';
                    }
                }
                // Checking if we have got a white space
                else if(fenPos[pos] == ' '){
                    whiteSpaceCounter ++;
                }
                //Otherwise we got an error - unexpected
                else{
                    // No error as there is no implementation for half move
                }
            }
            // Checking for full move number
            else if(whiteSpaceCounter == 5){
                if((fenPos[pos] - '0') > 0 && (fenPos[pos] - '0') <= 9){

                    //Checking if we have a two digit number
                    if((fenPos[pos+1] - '0') > 0 && (fenPos[pos+1] - '0') <= 9){
                        wholeMoves = (fenPos[pos] - '0')*10 + (fenPos[pos+1] - '0');
                        pos++;

                    }
                    // Otherwise we do one digit
                    else{
                        wholeMoves = fenPos[pos] - '0';
                    }


                    if(wholeMoves % 2 == 1 && whiteTurn){
                        // Everything correct we expect white turn and it is in an uneven wholeMove
                    }
                    else if (wholeMoves % 2 == 1 && !whiteTurn){
                        //ERROR: We did not expect white Turn = false
                    }
                    else if(wholeMoves % 2 == 0 && !whiteTurn){
                        // Everything correct we expect black turn and it is in an even wholeMove
                    }
                    else if(wholeMoves % 2 == 0 && whiteTurn){
                        //ERROR: We did not expect white Turn = true
                    }
                    else{
                        //ERROR: None of the cases got triggered!
                    }
                }
                // Checking for a white space
                else if(fenPos[pos] == ' '){
                    whiteSpaceCounter ++;
                }
                // Otherwise we got an error - unexpected
                else{
                    // Unexpected value for full move number
                }
            }
            //Otherwise unexpected whiteSpaces
            else{
                // We got unexpected number of white Spaces
            }
        }
    }


    // Updates the turn and wholeMoves and halfMoves
    void ChessBoard::fenVariableUpdate(int const &startPos, int const &endPos){
        //Switch colors turn
            whiteTurn = !whiteTurn;

            // If the cell we move to is occupied increase wholeMoves
            if(chessBoard.at(endPos).isOccupied()){
                wholeMoves ++;
                previousMoveWasPawn = false;
            }
            // If the chessPiece that has been moved is a pawn then increase wholeMoves
            else if(tolower(chessBoard.at(startPos).getChessPiece()->getName()) == 'p' && previousMoveWasPawn){
                wholeMoves ++;
                previousMoveWasPawn = false;
            }
            // If the chessPiece that has been moved is a pawn and previous was not a pawn Moved, then set previousPawn true and reset halfMoves
            else if(tolower(chessBoard.at(startPos).getChessPiece()->getName()) == 'p' && !previousMoveWasPawn){
                previousMoveWasPawn = true;
                halfMoves = 0;
            }
            // Otherwise we increase halfMoves
            else{
                //If previousPawn is true then we increase wholeMoves and reset previousPawn
                if(previousMoveWasPawn){
                    wholeMoves ++;
                    previousMoveWasPawn = false;
                }
                halfMoves ++;
            }
    }

    
    /*
        return values meaning:
             0 move is valid or is not a pawn
            -1 Pawn collided straight move
            -2 Pawn collided diagonal move
    */
    int ChessBoard::pawnMoveValid(Cell &startCell, Cell &endCell, vector<Cell> &chessBoardToCheck){

        // If we do not have a pawn return false
        if(tolower(startCell.getChessPieceName()) != 'p') return 0;


        // Checking if we have a straight move
        if(abs(startCell.getPosition()-endCell.getPosition())%8 == 0){


            //positive for black move and negative if white moves
            int move = 0;
            if(startCell.getChessPieceColor()) move = -1;
            else move = 1;


            // Checking all cells in the move direction
            for(int i = 1; i <= abs(move); i++){

                // Checking if the cell we would move through is occupied
                if(chessBoardToCheck.at(startCell.getPosition()+i*8*move).isOccupied()) return -1;
            }

        }
        // Otherwise we have a diagonal move
        else{

            //If the target square is not an opposing piece, dont allow move
            if(!(endCell.isOccupied()) || startCell.getChessPieceColor() == endCell.getChessPieceColor()){
                return -2;
            }
        }


        return 0;
    }

    /*
        return values meaning:
             0 move is valid or is not a king
            -6 King doesn't have castleRights King side
            -7 King doesn't have castleRights Queen side
    */
    int ChessBoard::castleMoveValid(Cell &startCell, Cell &endCell){
        // If we do not have a pawn return false
        if(tolower(startCell.getChessPieceName()) != 'k') return 0;

        bool isWhiteKing = startCell.getChessPieceColor();

        // Checking if we are castling left or right
        if(abs(endCell.getPosition()-startCell.getPosition())%2 == 0){
            int move = endCell.getPosition() - startCell.getPosition();

            // Checking if we are dealing with white king
            if(isWhiteKing){

                // If we do king side castle and we dont have castle Rights there return
                if(move == 2 && !castleRights[0][0]){
                    return -6;
                }
                // If we do queen side castle and we dont have castle Rights there we need to setHasMoved true as it wasn't set yet.
                if(move == -2 && !castleRights[0][1]){
                    return -7;
                }
            }
            // Otherwise we are dealing with black king
            else{
                // If we do king side castle and we dont have castle Rights there we need to setHasMoved true as it wasn't set yet.
                if(move == 2 && !castleRights[1][0]){
                    return -6;
                }
                // If we do queen side castle and we dont have castle Rights there we need to setHasMoved true as it wasn't set yet.
                if(move == -2 && !castleRights[1][1]){
                    return -7;
                }
            }
        }
        // Update castle Rights if still exist
        else{

            // Checking if we are dealing with white king
            if(isWhiteKing){

                // Checking if either side still has castle rights remove the castle rights
                if(castleRights[0][0] || castleRights[0][1]){
                    castleRights[0][0] = false;
                    castleRights[0][1] = false;
                }
            }
            // Otherwise we are dealing with black king
            else{

                // Checking if either side still has castle rights remove the castle rights
                if(castleRights[1][0] || castleRights[1][1]){
                    castleRights[1][0] = false;
                    castleRights[1][1] = false;
                }
            }
        }

        return 0;
    }

    /*
        return values meaning:
             0 no king in check
            -1 King in check
            -2 King in checkMate
    */
    int ChessBoard::kingCheck(int startPos, int endPos, bool colorToCheckWhite){

        vector<Cell> chessBoardCopy = chessBoard;

        // Doing the move on the copy to then check if the king will go into check
        chessBoardCopy.at(endPos).setChessPiece( chessBoardCopy.at(startPos).releaseChessPiece() );



        // Going through every cell in the board
        for(int cellPos = 0; cellPos < 63; cellPos++){

            Cell currentCell = chessBoardCopy.at(cellPos);


            // Checking if cell is Occupied and the color of the piece is the same as for the king we are checking
            if(currentCell.isOccupied() && currentCell.getChessPieceColor() != colorToCheckWhite){
                
                //Getting that chesspieces moveSet
                vector<MoveSet> moveSets = currentCell.getChessPiece()->getMoveSet();

                int multiCellMoveCount;

                // Checking if the piece can only move one cell, if yes set to 1 otherwise to 8
                if(currentCell.getChessPiece()->getOnlyMovesOneCell()) multiCellMoveCount = 1;
                else multiCellMoveCount = 8;

                // Creating variables for stretched movements and has collided
                int stretchedMove = 0;
                bool hasCollided;
                bool setKingInCheck = false;

                for(MoveSet ms : moveSets){

                    //Resetting that we have collided as we are looking through a new move set
                    hasCollided = false;

                    // If we already set the king in check with this piece then we can exit as each piece can only set the king in check once
                    if(setKingInCheck) break;


                    for(int moveCount = 1; moveCount <= multiCellMoveCount; moveCount++){
                        
                        //If we already collided in this moveSet then just skip logic
                        if(hasCollided) break;

                        //Calculating stretchedMove
                        stretchedMove = ms.getCellPos(moveCount, currentCell.getPosition());
                        // Checking if move not allowed
                        if(stretchedMove == -1) break;



                        // Checking if we are out of bounds
                        if(stretchedMove < 0 || stretchedMove > 63) break;

                        // Getting a copy of the looked at cell
                        Cell tempCell = chessBoardCopy.at(stretchedMove);


                        // Checking if the cell is occupied
                        if(tempCell.isOccupied()){

                            // Checking if the cell we look at is same color as ours. If yes then we have collided and cant move in that direction
                            if(tempCell.getChessPieceColor() == currentCell.getChessPieceColor()){
                                hasCollided = true;
                                break;
                            }

                            // Checking if the cell we look at is not same color and we are not looking at the king, If yes then we have collided
                            if(tempCell.getChessPieceColor() != currentCell.getChessPieceColor() && tolower(tempCell.getChessPieceName()) != 'k'){
                                hasCollided = true;
                                break;
                            }

                            // Checking if the cell we look at is not same color and we are looking at the king
                            if(tempCell.getChessPieceColor() != currentCell.getChessPieceColor() && tolower(tempCell.getChessPieceName()) == 'k'){

                                // Checking if pawnMoveValid
                                int returnedPawnMoveValid = pawnMoveValid(currentCell, tempCell, chessBoardCopy);

                                //If we do not have a valid pawn move then break out of this for loop
                                if(returnedPawnMoveValid != 0) break;

                                return -1;
                            }
                        }
                    }
                }

            }
        }
        
        // Otherwise king is not in Check
        return 0;
    }

    /*
        return values meaning:
             0 move is valid
            -1 Pawn collided straight move
            -2 Pawn collided diagonal move
            -3 starting Cell is empty, thus no moveable piece
            -4 not that colors turn
            -5 went through all moveSets and did not find a correct move thus move invalid or it was blocked by own color
            -6 King doesn't have castleRights King side
            -7 King doesn't have castleRights Queen side
    */
    void ChessBoard::castle(string const &moveKing, string const &moveRook, int &startPos, int &endPos, int &isMoveValidCode){
        //Checking if king move is valid
        
        getVecPosFromMove(moveKing, startPos, endPos);
        isMoveValidCode = isMoveValid(startPos, endPos);

        // If King move is valid then
        if(isMoveValidCode == 0){
            int startPosRook = 0;
            int endPosRook = 0;

            // Check if rook move is valid
            getVecPosFromMove(moveRook, startPosRook, endPosRook);
            isMoveValidCode = isMoveValid(startPosRook, endPosRook);

            // If rook move valid then place rook and remove castle rights
            if(isMoveValidCode == 0) {
                chessBoard.at(endPosRook).setChessPiece( chessBoard.at(startPosRook).releaseChessPiece() );
            }
            else{
                // Otherwise reset king has moved
                chessBoard.at(startPos).getChessPiece()->setHasMoved(false);
            }
        }
    }


    /*
        return values meaning:
             0 move is valid
            -1 Pawn collided straight move
            -2 Pawn collided diagonal move
            -3 starting Cell is empty, thus no moveable piece
            -4 not that colors turn
            -5 went through all moveSets and did not find a correct move thus move invalid or it was blocked by own color
            -6 King doesn't have castleRights King side
            -7 King doesn't have castleRights Queen side
            -8 Own king in check
            -9 Other King in check mate
    */
    int ChessBoard::isMoveValid(int startPos, int endPos){


        Cell &startCell = chessBoard.at(startPos);
        Cell &endCell = chessBoard.at(endPos);


        // Checking if startCell is empty, if yes return -3
        if(!startCell.isOccupied()) return -3;

        // Check if it is that colors turn, if not return -4
        if(!(whiteTurn == startCell.getChessPieceColor())) return -4;

        //Getting startCell chessPiece information
        std::shared_ptr<ChessPiece> chessPiece = startCell.getChessPiece();
        vector<MoveSet> moveSets = chessPiece->getMoveSet();

        int multiCellMoveCount;

        // Checking if the piece can only move one cell, if yes 
        if(chessPiece->getOnlyMovesOneCell()) multiCellMoveCount = 1;
        else multiCellMoveCount = 8;

        // Creating variables for stretched movements and has collided
        int stretchedMove = 0;
        bool hasCollided;


        //Loop each move in moveSet
        for(MoveSet ms : moveSets){
            
            //Resetting that we have collided as we are looking through a new move set
            hasCollided = false;

            //Loop through all the cells it can move through
            for(int moveCount = 1; moveCount <= multiCellMoveCount; moveCount++){

                //If we already collided in this moveSet then just skip logic
                if(hasCollided) break;

                //Calculating stretchedMove
                stretchedMove = ms.getCellPos(moveCount, startCell.getPosition());
                // Checking if move not allowed
                if(stretchedMove == -1) break;

                // Getting a reference to the looked at cell
                Cell tempCell = chessBoard.at(stretchedMove);

                // Checking if the cell is occupied
                if(tempCell.isOccupied()){

                    // Checking if the cell we move through is same color as ours. If yes then we have collided and cant move in that direction
                    if(tempCell.getChessPieceColor() == startCell.getChessPieceColor()){
                        hasCollided = true;
                        break;
                    }

                    // Checking if the cell we move through is not same as color and we are not at the wanted reached position. If yes we have collided
                    if(tempCell.getChessPieceColor() != startCell.getChessPieceColor() && tempCell.getPosition() != endCell.getPosition()){
                        hasCollided = true;
                        break;
                    }
                }


                // Checking if our move is looked at Cell is our expected Cell
                if(tempCell.getPosition() == endCell.getPosition()){
                    
                    // Checking if pawnMoveValid
                    int returnedPawnMoveValid = pawnMoveValid(startCell, endCell, chessBoard);

                    // If we do not have a valid pawn move then return the error code
                    if(returnedPawnMoveValid != 0) return returnedPawnMoveValid;

                    // Checking if we have a valid castleMove
                    int returnedCastleMoveValid = castleMoveValid(startCell, endCell);

                    // If we do not have a valid castle move then return the error code
                    if(returnedCastleMoveValid != 0) return returnedCastleMoveValid; 

                    //Checking if we set our own king in check
                    int returnedOwnKingInCheck = kingCheck(startCell.getPosition(), endCell.getPosition(), startCell.getChessPieceColor());

                    if(returnedOwnKingInCheck == -1){
                        return -8;
                    }

                    //Checking if we set the other king in check
                    int returnedOppositeKingInCheck = kingCheck(startCell.getPosition(), endCell.getPosition(), !startCell.getChessPieceColor());

                    if(returnedOppositeKingInCheck == -1){

                        // Need to check if he is in check mate!
                        //int inCheckMate = kingCheck(startCell.getPosition(), endCell.getPosition(), !startCell.getChessPieceColor(), true, chessBoard);
                        int inCheckMate = 0;
                        if(inCheckMate == -2){
                            return -9;
                        }
                    }


                    if(startCell.getChessPieceColor()) whiteKingInCheck = false;
                    else blackKingInCheck = false;

                    //Otherwise it is correct move or not a pawn or correct move or not a king
                    //Need to check if piece is a pawn if yes need to do extra checks
                    chessPiece->setHasMoved(true);

                    return 0;
                }
            }
        }

        return -5;
        
    }


    void ChessBoard::getVecPosFromMove(string const &move, int &startPos, int &endPos){

        int rowMoveFrom = (int)move[0] - 97;
        int columnMoveFrom = 7 - (move[1] - '1');
        int rowMoveTo = (int)move[2] - 97;
        int columnMoveTo = 7 - (move[3] - '1');

        startPos = (columnMoveFrom*8) + rowMoveFrom;
        endPos = (columnMoveTo*8) + rowMoveTo;
    }

    /*
        return values meaning:
             0 move is valid
            -1 Pawn collided straight move
            -2 Pawn collided diagonal move
            -3 starting Cell is empty, thus no moveable piece
            -4 not that colors turn
            -5 went through all moveSets and did not find a correct move thus move invalid or it was blocked by own color
            -6 King doesn't have castleRights King side
            -7 King doesn't have castleRights Queen side
            -8 Own king in check
            -9 Other King in check mate
            -10 Pawn not allowed to move there as only promotion move is allowed
    */
    int ChessBoard::processMove(string const &move){

        int startPos = 0;
        int endPos = 0;
        int isMoveValidCode = 0;

        getVecPosFromMove(move, startPos, endPos);

        // Checking if the piece is a pawn and we move into a end position then return error because we are only allowed to move into when promoting
        if(tolower(chessBoard.at(startPos).getChessPieceName()) == 'p' && ((0 <= endPos && endPos <= 7) || (56 <= endPos && endPos <= 63))){
            return -10;
        }


        // Checking if move is a castling move

        //White Castle King side and we have castleRights
        if(move.compare("e1g1") == 0 && castleRights[0][0]){

            // Trying to castle
            castle("e1g1", "h1f1", startPos, endPos, isMoveValidCode);

            // If successful then we remove castleRights
            if(isMoveValidCode == 0){
                castleRights[0][0] = false;
                castleRights[0][1] = false;
                lastMoveCastle = true;
            }
        }
        //White Castle Queen side
        else if(move.compare("e1c1") == 0 && castleRights[0][1]){

            // Trying to castle
            castle("e1c1", "a1d1", startPos, endPos, isMoveValidCode);

            // If successful then we remove castleRights
            if(isMoveValidCode == 0){
                castleRights[0][0] = false;
                castleRights[0][1] = false;
                lastMoveCastle = true;
            }
        }
        //Black Castle King side
        else if(move.compare("e8g8") == 0 && castleRights[1][0]){
            // Trying to castle
            castle("e8g8", "h8f8", startPos, endPos, isMoveValidCode);
            // If successful then we remove castleRights
            if(isMoveValidCode == 0){
                castleRights[1][0] = false;
                castleRights[1][1] = false;
                lastMoveCastle = true;
            }
        }
        //Black Castle Queen side
        else if(move.compare("e8c8") == 0){
            // Trying to castle
            castle("e8c8", "a8d8", startPos, endPos, isMoveValidCode);
            // If successful then we remove castleRights
            if(isMoveValidCode == 0){
                castleRights[1][0] = false;
                castleRights[1][1] = false;
                lastMoveCastle = true;
            }
        }
        //Otherwise not castling
        else{
            isMoveValidCode = isMoveValid(startPos, endPos);
            lastMoveCastle = false;
        }


        // If move is valid
        if(isMoveValidCode == 0){
            
            //Updating colorTurn, halfMoves and wholeMoves
            fenVariableUpdate(startPos, endPos);

            // Updating castle Rights if needed
            // Checking if piece that was moved was the White Rook
            if(chessBoard.at(startPos).getChessPiece()->getName() == 'R'){
                // Checking if it was the rook king side and he still has castleRights
                if(startPos == 63 && castleRights[0][0]){
                    castleRights[0][0] = false;
                }
                // Checking if it was the rook queen side and he still has castleRights
                else if(startPos == 56 && castleRights[0][1]){
                    castleRights[0][1] = false;
                }
            }
            // Checking if piece that was moved was the Black Rook
            else if(chessBoard.at(startPos).getChessPiece()->getName() == 'r'){
                // Checking if it was the rook king side and he still has castleRights
                if(startPos == 7 && castleRights[1][0]){
                    castleRights[1][0] = false;
                }
                // Checking if it was the rook queen side and he still has castleRights
                else if(startPos == 0 && castleRights[1][1]){
                    castleRights[1][1] = false;
                }
            }

            chessBoard.at(endPos).setChessPiece( chessBoard.at(startPos).releaseChessPiece() );

            return 0;
        }
        
        return isMoveValidCode;
        
    }

    /*
        return values meaning:
             0 move is valid
            -1 Pawn collided straight move
            -2 Pawn collided diagonal move
            -3 starting Cell is empty, thus no moveable piece
            -4 not that colors turn
            -5 went through all moveSets and did not find a correct move thus move invalid or it was blocked by own color
            -6 King doesn't have castleRights King side
            -7 King doesn't have castleRights Queen side
            -8 Own king in check
            -11 The piece is not a pawn thus cannot be used for promotion
            -12 Pawn is not moving into endPosition and cannot be used for promotion
            -13 Invalid piece name to promote into
    */
    int ChessBoard::processPromotionMove(string const &move){
        int startPos = 0;
        int endPos = 0;
        int isMoveValidCode = 0;
        char pieceToCreate = move[4];
        ChessPiece* chessPiecePromoted;


        getVecPosFromMove(move, startPos, endPos);

        //Checking if the piece to move is a pawn
        if(tolower(chessBoard.at(startPos).getChessPieceName()) == 'p'){

            // Checking if we are going to the end of the board
            if((0 <= endPos && endPos <= 7) || (56 <= endPos && endPos <= 63)){

                //Check if the pawn move is valid
                isMoveValidCode = isMoveValid(startPos, endPos);

                if(isMoveValidCode == 0){

                    // Checking which piece to create and in which color
                    switch (pieceToCreate)
                    {
                        case 'r':
                            // Checking color of initial pawn and create that colors piece
                            if(chessBoard.at(startPos).getChessPieceColor()) chessPiecePromoted = new Rook(true);
                            else chessPiecePromoted = new Rook(false);

                            // So that it is not allowed to castle
                            chessPiecePromoted->setHasMoved(true);
                            break;

                        case 'n':
                            if(chessBoard.at(startPos).getChessPieceColor()) chessPiecePromoted = new Knight(true);
                            else chessPiecePromoted = new Knight(false);
                            break;

                        case 'b':
                            if(chessBoard.at(startPos).getChessPieceColor()) chessPiecePromoted = new Bishop(true);
                            else chessPiecePromoted = new Bishop(false);
                            break;

                        case 'q':
                            if(chessBoard.at(startPos).getChessPieceColor()) chessPiecePromoted = new Queen(true);
                            else chessPiecePromoted = new Queen(false);
                            break;
                        
                        default:
                            return -13;
                            break;
                    }

                    //Updating colorTurn, halfMoves and wholeMoves
                    fenVariableUpdate(startPos, endPos);

                    //Setting new promoted piece
                    chessBoard.at(startPos).setChessPiece(NULL);
                    chessBoard.at(endPos).setChessPiece(chessPiecePromoted);

                    return 0;

                }
                else{
                    return isMoveValidCode;
                }
            }
            // Otherwise we do not move to the end of the board so -12
            else{
                return -12;
            }
        }

        // Otherwise not a pawn return -11 code
        return -11; 
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


    /*
        return values meaning:
             0 move is valid
            -1 Pawn collided straight move
            -2 Pawn collided diagonal move
            -3 starting Cell is empty, thus no moveable piece
            -4 not that colors turn
            -5 went through all moveSets and did not find a correct move thus move invalid or it was blocked by own color
            -6 King doesn't have castleRights King side
            -7 King doesn't have castleRights Queen side
            -8 Own king in check
            -9 Other King in check mate
            -10 Pawn not allowed to move there as only promotion move is allowed
            -11 The piece is not a pawn thus cannot be used for promotion
            -12 Pawn is not moving into endPosition and cannot be used for promotion
            -13 Invalid piece name to promote into
            -14 Inputted move does not correspond format
    */
    int ChessBoard::move(string &move){

        transform(move.begin(), move.end(), move.begin(), ::tolower);

        if(DataChecker::isCorrectMoveFormat(move)){

            int returnedProcessMove = processMove(move);
            return returnedProcessMove;

        }
        else if(DataChecker::isCorrectMoveFormatPromotion(move)){

            int returnedProcessPromotionMove = processPromotionMove(move);

            return returnedProcessPromotionMove;
        }
        else{
            return -14;
        }
    }

    string ChessBoard::toFENString(){
        int counterEmpty = 0;
        char piece;
        string fenPos = "";

        for(int position = 0; position < 64; position++){


            // Getting the name of the chessPiece at the position
            piece = chessBoard.at(position).getChessPieceName();

            // Check if current position is a multiple of 8, if yes then add FEN separator
            if(position % 8 == 0){
                 fenPos += "/";
            }

            // Check if we have an empty position then increase counter
            if(piece == '-' && counterEmpty >= 0){
                //Checking if we have an empty position and we are at the end of a column then we need
                //to reset counter and print the so far read empty Strings number
                if(position%8 == 7){
                    fenPos += to_string(counterEmpty+1);
                    counterEmpty = 0;
                }
                else{
                    counterEmpty++;
                }
                
            }
            // Check if we have a piece again and counter greater 0 then add the counter to it
            else if(piece != '-' && counterEmpty > 0){
                fenPos += to_string(counterEmpty);
                fenPos += piece;
                counterEmpty = 0;
            }
            else{
                fenPos += piece;
            }
        }

        fenPos += " ";

        // Adding whose move turn it is
        if(whiteTurn) fenPos += "w";
        else fenPos += "b";

        fenPos += " ";

        if(castleRights[0][0] || castleRights[0][1] || castleRights[1][0] || castleRights[1][1]){
            // Adding Castle right for white king side if still possible
            if(castleRights[0][0]) fenPos += "K";
            // Adding Castle right for white queen side if still possible
            if(castleRights[0][1]) fenPos += "Q";
            // Adding Castle right for black king side if still possible
            if(castleRights[1][0]) fenPos += "k";
            // Adding Castle right for black queen side if still possible
            if(castleRights[1][1]) fenPos += "q";

        }else fenPos += "-";

        fenPos += " - ";
        fenPos += to_string(halfMoves);
        fenPos += " ";
        fenPos += to_string(wholeMoves);
        fenPos.erase(0,1);


        return fenPos;
    }

    string ChessBoard::toString(){
        string temp = "";

        for(int i = 0; i < 64; i++){

            if(i%8 == 0) temp += "\n";
            string chessPieceName(1, chessBoard.at(i).getChessPieceName());

            temp+= "[" +  chessPieceName + "]";

        }

        return temp;
    }