#include "ibo1_IRC_API/Utility/DataChecker.h"

    // ///////////// //
    // Constructors. //
    // ///////////// //

    // ////////////// //
    // Class methods. //
    // ////////////// //

    // ////////////////////// //
    // Read/Write properties. //
    // ////////////////////// //

    // ///////////////////// //
    // Read-only properties. //
    // ///////////////////// //

    // //////// //
    // Methods. //
    // //////// //

    bool DataChecker::isCorrectMove(string const& moveString){
        regex r("[1-8][a-h][1-8][a-h]");
        smatch match;
        if(regex_search(moveString,match, r)) return true;
        return false;
    }