#include "ibo1_IRC_API/Chess.h"

    // ///////////// //
    // Constructors. //
    // ///////////// //

    Chess::Chess(){
        
    }

    Chess::Chess(string processFilePathName){
        SubProcessHandler subProcessHandler(processFilePathName);
    }

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