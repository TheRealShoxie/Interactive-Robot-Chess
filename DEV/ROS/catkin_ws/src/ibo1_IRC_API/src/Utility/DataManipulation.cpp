#include "ibo1_IRC_API/DataManipulation.h"

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

    string DataManipulation::subString(string &stringToExtract, string splitter){
        int end;
        string returnString;

        end = stringToExtract.find(splitter);
        returnString = stringToExtract.substr(0, end);
        stringToExtract.erase(stringToExtract.begin(), stringToExtract.begin() + end + splitter.size());

        return returnString;
    }