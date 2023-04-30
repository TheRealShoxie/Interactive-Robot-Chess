#ifndef SUBPROCESSHANDLER_H
#define SUBPROCESSHANDLER_H

/*
 * SubProcessHandler File
 * <p>
 * The SubProcessHandler file defines the SubProcessHandler class, the SubProcess Class and cpipe class.
 * 
 * <p>
 * cpipe class:
 * 
 * 3rd party code is used in this class. It was the created by github user konstantin from following link: https://gist.github.com/konstantint/d49ab683b978b3d74172
 * It was taken on the day 20.03.2023
 * This class is the original code
 * 
 * Used for linux piping.
 * 
 * 
 * <p>
 * SubProcess class:
 * 
 * 3rd party code is used in this class. It was the created by github user konstantin from following link: https://gist.github.com/konstantint/d49ab683b978b3d74172
 * It was taken on the day 20.03.2023
 * Its current implementation has differently named class name and was expanded on with the help of Maxim Buzdalov from lines 58-62.
 * 
 * Its usage is for creating child processes
 * 
 * 
 * <p>
 * SubProcessHandler class:
 * 
 * A class written to interact with the subProcess created. Main Implementation is for usage in starting chessEngines. Should be able to start any kind of process though
 * It is used inside of UCIHandler.h
 * 
 * 
 * @author Omar Ibrahim
 * @version 0.1 ( Initial development ).
 * @version 1.0 ( Initial release ).
 * 
 * @see UCIHandler.h
*/

    // ////////// //
    // Includes.  //
    // ////////// //

#include <ext/stdio_filebuf.h> // NB: Specific to libstdc++
#include <sys/wait.h>
#include <string.h>
#include <unistd.h>
#include <iostream>
#include <memory>
#include <exception>
#include <istream>
#include <vector>

using namespace std;


/*
    ---------------------------------------------------------------------------------------------
*/
class cpipe {
private:
    int fd[2];
public:
    const inline int read_fd() const { return fd[0]; }
    const inline int write_fd() const { return fd[1]; }
    cpipe() { if (pipe(fd)) throw runtime_error("Failed to create pipe"); }
    void close() { ::close(fd[0]); ::close(fd[1]); }
    ~cpipe() { close(); }
};

/*
    ---------------------------------------------------------------------------------------------
*/
class SubProcess {
private:
    cpipe write_pipe;
    cpipe read_pipe;
public:
    int child_pid = -1;
    unique_ptr<__gnu_cxx::stdio_filebuf<char> > write_buf = NULL; 
    unique_ptr<__gnu_cxx::stdio_filebuf<char> > read_buf = NULL;
    ostream stdin;
    istream stdout;

    SubProcess(): stdin(NULL), stdout(NULL){}

    SubProcess(vector<string> const &argvSource, bool with_path = false, const char* const envp[] = 0): stdin(NULL), stdout(NULL){
        child_pid = fork();
        if (child_pid == -1) throw runtime_error("Failed to start child process"); 
        if (child_pid == 0) {   // In child process
            dup2(write_pipe.read_fd(), STDIN_FILENO);
            dup2(read_pipe.write_fd(), STDOUT_FILENO);
            write_pipe.close(); read_pipe.close();
            int result;

            char ** argv = new char*[argvSource.size() + 1];
            for (size_t i = 0; i < argvSource.size(); ++i) {
                argv[i] = strdupa(argvSource[i].c_str());
            }
            argv[argvSource.size()] = NULL;

            if (with_path) {
                if (envp != 0) result = execvpe(argv[0], const_cast<char* const*>(argv), const_cast<char* const*>(envp));
                else result = execvp(argv[0], const_cast<char* const*>(argv));
            }
            else {
                if (envp != 0) result = execve(argv[0], const_cast<char* const*>(argv), const_cast<char* const*>(envp));
                else result = execv(argv[0], const_cast<char* const*>(argv));
            }
            if (result == -1) {
               // Note: no point writing to stdout here, it has been redirected
               throw runtime_error("Failed to launch program");
               cerr << "Error: Failed to launch program" << endl;
               exit(1);
            }
        }
        else {
            close(write_pipe.read_fd());
            close(read_pipe.write_fd());
            write_buf = unique_ptr<__gnu_cxx::stdio_filebuf<char> >(new __gnu_cxx::stdio_filebuf<char>(write_pipe.write_fd(), ios::out));
            read_buf = unique_ptr<__gnu_cxx::stdio_filebuf<char> >(new __gnu_cxx::stdio_filebuf<char>(read_pipe.read_fd(), ios::in));
            stdin.rdbuf(write_buf.get());
            stdout.rdbuf(read_buf.get());
        }
    }
    
    void send_eof() { write_buf->close(); }
    
    int wait() {
        int status;
        waitpid(child_pid, &status, 0);
        return status;
    }
};



   
/*
    ---------------------------------------------------------------------------------------------
*/   

class SubProcessHandler{

    public:

        // ///////////// //
        // Constructors. //
        // ///////////// //

        // Default constructor
        SubProcessHandler(){}

        // Constructor which specifies a filepath to a process to start or a command
        SubProcessHandler(string const &processFilePathName)
            : subProcess({processFilePathName}) {
        }

        // //////// //
        // Methods. //
        // //////// //

        // Gets the stdout line from the process that has been started
        void getLine(string &returnedLine){
            
            // Checking if the child pid is not -1
            if(subProcess.child_pid == -1) return;
            getline(subProcess.stdout, returnedLine);
        }

        // Writes to the stdin  of the started process
        void write(string const &message){

            // Checking if the child pid is not -1
            if(subProcess.child_pid == -1) return;
            subProcess.stdin << message << endl;
        }


        // Deconstructor for SubProcessHandler
        ~SubProcessHandler() {

            // Checking if the child pid is not -1
            if(subProcess.child_pid == -1) return;
            subProcess.send_eof();

            cout << "Waiting to terminate..." << endl;
            cout << "Status: " << subProcess.wait() << endl;
        }

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
        
        // //////////////// //
        // Class variables. //
        // //////////////// //

        // /////////////////// //
        // Instance variables. //
        // /////////////////// //

        SubProcess subProcess;
};

#endif //SUBPROCESSHANDLER_H