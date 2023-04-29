#include "iostream"

using namespace std;

// Datatype
typedef uint8_t BYTE;


// Receiver
#define SENDER_SYSTEMSTATEMACHINE                                   (BYTE)0x00
#define SENDER_SERVER                                               (BYTE)0x01
#define SENDER_CHESSWRAPPER                                         (BYTE)0x02
#define SENDER_CREATETARGET                                         (BYTE)0x03
#define SENDER_ROBOTARMSTATEMACHINE                                 (BYTE)0x04


// Protocol Commands
//CreateTarget Cmds
#define CMD_INTERNAL_SETTARGET                                      (BYTE)0x01
#define CMD_INTERNAL_CLEARTARGET                                    (BYTE)0x02
#define CMD_INTERNAL_ROBOTARMMOVE                                   (BYTE)0x03


// Chess CMDs
#define CMD_INTERNAL_GETCHESSENGINES                                (BYTE)0x04
#define CMD_INTERNAL_STARTCHESSENGINE                               (BYTE)0x05
#define CMD_INTERNAL_STOPCHESSENGINE                                (BYTE)0x06
#define CMD_INTERNAL_GETCHESSENGINEOPTIONS                          (BYTE)0x07
#define CMD_INTERNAL_SETCHESSENGINEOPTIONS                          (BYTE)0x08
#define CMD_INTERNAL_SETSEARCHOPTIONS                               (BYTE)0x09
#define CMD_INTERNAL_PLAYERMOVE                                     (BYTE)0x0a
#define CMD_INTERNAL_CHESSENGINEMOVE                                (BYTE)0x0b
#define CMD_INTERNAL_LASTMOVECASTLEMOVE                             (BYTE)0x0c
#define CMD_INTERNAL_SYSTEMWITHOUTSIM                               (BYTE)0x0d
#define CMD_INTERNAL_SYSTEMFULLSIM                                  (BYTE)0x0e


// Error Codes:
#define ERROR_INTERNAL_CMD_UNRECOGNIZABLE                           (BYTE)0xfe

//Chess Errors
#define ERROR_INTERNAL_CMD_CHESSENGINEDOESNTEXIST                   (BYTE)0xf9
#define ERROR_INTERNAL_CMD_CHESSENGINENOTSTARTED                    (BYTE)0xf8
#define ERROR_INTERNAL_CMD_NOCHESSENGINERUNNING                     (BYTE)0xf7
#define ERROR_INTERNAL_CMD_CHESSENGINEOPTIONDOESNTEXIST             (BYTE)0xf6
#define ERROR_INTERNAL_CMD_SEARCH_OPTION_DOESNT_EXIST               (BYTE)0xf5
#define ERROR_INTERNAL_CMD_PAWNCOLLIDEDSTRAIGHT                     (BYTE)0xf4
#define ERROR_INTERNAL_CMD_PAWNCOLLIDEDDIAGONALOREMPTYCELL          (BYTE)0xf3
#define ERROR_INTERNAL_CMD_STARTINGCELLEMPTY                        (BYTE)0xf2
#define ERROR_INTERNAL_CMD_NOTTHATCOLORSTURN                        (BYTE)0xf1
#define ERROR_INTERNAL_CMD_MOVEINVALIDORBLOCKEDBYSAMECOLOR          (BYTE)0xf0
#define ERROR_INTERNAL_CMD_CANNOTCASTLEKINGSIDE                     (BYTE)0xef
#define ERROR_INTERNAL_CMD_CANNOTCASTLEQUEENSIDE                    (BYTE)0xee
#define ERROR_INTERNAL_CMD_OWNKINGINCHECK                           (BYTE)0xed
#define ERROR_INTERNAL_CMD_OTHERKINGINCHECKMATE                     (BYTE)0xec
#define ERROR_INTERNAL_CMD_PAWNNOTALLOWEDNOTPROMOTIONMOVE           (BYTE)0xeb
#define ERROR_INTERNAL_CMD_PIECETOPROMOTEISNOTPAWN                  (BYTE)0xea
#define ERROR_INTERNAL_CMD_PAWNNOTMOVINGTOENDOFBOARD                (BYTE)0xe9
#define ERROR_INTERNAL_CMD_INVALIDPIECENAMETOPROMOTEINTO            (BYTE)0xe8
#define ERROR_INTERNAL_CMD_CHESSENGINECREATEDNOMOVE                 (BYTE)0xe7
#define ERROR_INTERNAL_CMD_INVALIDMOVEFORMAT                        (BYTE)0xe6
#define ERROR_INTERNAL_CMD_NOCHESSBOARDINFORMATION                  (BYTE)0xe5
#define ERROR_INTERNAL_CMD_PICKUPCELLEMPTY                          (BYTE)0xe4
#define ERROR_INTERNAL_CMD_SYSTEMINPLAYCHESSSTATEMACHINE            (BYTE)0xe3
#define ERROR_INTERNAL_CMD_LASTMOVEWASNOTCASTLEMOVE                 (BYTE)0xe2

//Create Target errors


