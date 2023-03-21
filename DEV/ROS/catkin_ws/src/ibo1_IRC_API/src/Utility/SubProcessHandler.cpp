#include "ibo1_IRC_API/SubProcessHandler.h"

    // ///////////// //
    // Constructors. //
    // ///////////// //

    SubProcessHandler::SubProcessHandler(){}

    SubProcessHandler::SubProcessHandler(string processFilePathName){
        const char* const argv[] = {processFilePathName.c_str(), (const char*)0};
        SubProcess subProcess(argv);
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

    string SubProcessHandler::getLine(){
        if(subProcess.child_pid == -1) return "-1";
        string temp;
        getline(subProcess.stdout, temp);
        return temp;
    }

    void SubProcessHandler::write(string message){
        if(subProcess.child_pid == -1) return;
        subProcess.stdin << message << endl;
    }

    void SubProcessHandler::closeSubProcess(){
        if(subProcess.child_pid == -1) return;
        subProcess.send_eof();
        cout << "Waiting to terminate..." << endl;
        cout << "Status: " << subProcess.wait() << endl;
    }