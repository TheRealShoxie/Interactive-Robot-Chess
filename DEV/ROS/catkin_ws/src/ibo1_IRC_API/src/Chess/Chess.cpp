#include "ibo1_IRC_API/Chess.h"

    // ///////////// //
    // Constructors. //
    // ///////////// //

    Chess::Chess(){}
    Chess::Chess(string const &processFilePathName)
        :uciHandler(processFilePathName), searchOptions("depth 10"),
        fenBoardState("rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1"){
    }

    // ////////////// //
    // Class methods. //
    // ////////////// //

    // ////////////////////// //
    // Read/Write properties. //
    // ////////////////////// //

    void Chess::setSearchOptions(string const &setSearchOptions){
        searchOptions = setSearchOptions;
    }

    void Chess::getSearchOptions(string &getSearchOptions){
        getSearchOptions = searchOptions;
    }

    // ///////////////////// //
    // Read-only properties. //
    // ///////////////////// //

    // //////// //
    // Methods. //
    // //////// //