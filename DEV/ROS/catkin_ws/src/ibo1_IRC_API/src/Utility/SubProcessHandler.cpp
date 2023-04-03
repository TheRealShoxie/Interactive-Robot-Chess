#include <ibo1_IRC_API/Utility/SubProcessHandler.h>

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
    }

    void SubProcessHandler::write(string const &message){
        cout << "Memory Address SubProcessor: " << &subProcess << endl;
        cout << "Asking for child pid: ";
        cout << (int)subProcess.child_pid << endl;
        cout << "Message to be printed: " << message << endl;

        if(subProcess.child_pid == -1) return;
        subProcess.stdin << message << endl;
    }

    void SubProcessHandler::closeSubProcess(){
        if(subProcess.child_pid == -1) return;
        subProcess.send_eof();

        cout << "Waiting to terminate..." << endl;
        cout << "Status: " << subProcess.wait() << endl;
    }