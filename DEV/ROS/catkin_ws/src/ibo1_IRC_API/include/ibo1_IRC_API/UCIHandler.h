#ifndef UCIHANDLER_H
#define UCIHANDLER_H

#include <ibo1_IRC_API/SubProcessHandler.h>
#include <ibo1_IRC_API/DataManipulation.h>
   
    // ////////// //
    // Structs.   //
    // ////////// //

    struct EngineOption{
        string name = "";
        string typeOfValue;
        string defaultValue;
        string minValue;
        string maxValue;
        string restValues;

        inline bool operator==(const EngineOption& e) const{
            return (name == e.name && typeOfValue == e.typeOfValue && defaultValue == e.defaultValue && minValue == e.minValue && maxValue == e.maxValue && restValues == e.restValues);
        }
    };

    // bool operator==(const EngineOption& lhs, const EngineOption& rhs){
    //     return lhs.name == rhs.name && lhs.typeOfValue == rhs.typeOfValue && lhs.defaultValue == rhs.defaultValue && lhs.minValue == rhs.minValue && lhs.maxValue == rhs.maxValue && lhs.restValues == rhs.restValues;
    // }

    // ////////// //
    // Constants. //
    // ////////// //


class UCIHandler{

    public:

        // ///////////// //
        // Constructors. //
        // ///////////// //
        UCIHandler();
        UCIHandler(string const &processFilePathName);

        // //////// //
        // Methods. //
        // //////// //

        bool isEngineReady();
        bool startUCI(vector<EngineOption> &engineOptions);

        string makeMove(string const &fenPosition, string const &searchSettings);
        void setEngineOptions(string const &optionName, string const &value);
        bool startNewGame();
        void closeProcess();

        // Moved here for testing
        EngineOption createEngineOption(string &returnedLine);


        // ////////////////////// //
        // Read/Write properties. //
        // ////////////////////// //

        // ///////////////////// //
        // Read-only properties. //
        // ///////////////////// //

    private:

        // ////////////// //
        // Class methods. //
        // ////////////// //
        //EngineOption createEngineOption(string &returnedLine);

        // //////////////// //
        // Class variables. //
        // //////////////// //

        // /////////////////// //
        // Instance variables. //
        // /////////////////// //
        SubProcessHandler subProcessHandler;
        DataManipulation dm;

    
};
#endif //UCIHANDLER_H