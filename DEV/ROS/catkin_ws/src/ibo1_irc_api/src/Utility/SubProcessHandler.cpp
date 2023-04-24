#include <ibo1_irc_api/Utility/SubProcessHandler.h>

    // ///////////// //
    // Constructors. //
    // ///////////// //

    SubProcessHandler::SubProcessHandler(){}

    //Written by maxim
    SubProcessHandler::SubProcessHandler(string const &processFilePathName)
        : subProcess({processFilePathName}) {
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

    void SubProcessHandler::getLine(string &returnedLine){
        if(subProcess.child_pid == -1) return;
        getline(subProcess.stdout, returnedLine);
        //if(returnedLine.length() > 0) cout << returnedLine << endl;
    }

    void SubProcessHandler::write(string const &message){
        if(subProcess.child_pid == -1) return;
        subProcess.stdin << message << endl;
    }

    SubProcessHandler::~SubProcessHandler() {
        if(subProcess.child_pid == -1) return;
        subProcess.send_eof();

        cout << "Waiting to terminate..." << endl;
        cout << "Status: " << subProcess.wait() << endl;
    }