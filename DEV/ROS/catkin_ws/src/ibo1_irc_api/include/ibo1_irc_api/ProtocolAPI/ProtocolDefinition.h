#include "iostream"

using namespace std;

// Datatype
typedef uint8_t BYTE;

// Protocol Commands
#define CMD_DISCONNECT                                  (BYTE)0x00
#define CMD_CONNECT                                     (BYTE)0x01
#define CMD_LOGIN                                       (BYTE)0x02
#define CMD_CREATEUSER                                  (BYTE)0x03
#define CMD_GETCHESSENGINES                             (BYTE)0x04
#define CMD_STARTCHESSENGINE                            (BYTE)0x05
#define CMD_STOPCHESSENGINE                             (BYTE)0x06
#define CMD_GETCHESSENGINEOPTIONS                       (BYTE)0x07
#define CMD_SETCHESSENGINEOPTIONS                       (BYTE)0x08
#define CMD_SETSEARCHOPTIONS                            (BYTE)0x09
#define CMD_PLAYERMOVE                                  (BYTE)0x0a
#define CMD_CHESSENGINEMOVE                             (BYTE)0x0b
#define CMD_SYSTEMWITHOUTSIM                            (BYTE)0x0c
#define CMD_SYSTEMFULLSIM                               (BYTE)0x0d


// Error Codes:
#define ERROR_CMD_READINGBYTES                          (BYTE)0xff
#define ERROR_CMD_UNRECOGNIZABLE                        (BYTE)0xfe
#define ERROR_CONNECT                                   (BYTE)0xfd
#define ERROR_CMD_USERDOESNTEXIST                       (BYTE)0xfc
#define EROR_CMD_USERALREADYEXISTS                      (BYTE)0xfb
#define ERROR_CMD_USERNOTCREATED                        (BYTE)0xfa
#define ERROR_CMD_CHESSENGINEDOESNTEXIST                (BYTE)0xf9
#define ERROR_CMD_CHESSENGINENOTSTARTED                 (BYTE)0xf8
#define ERROR_CMD_NOCHESSENGINERUNNING                  (BYTE)0xf7
#define ERROR_CMD_CHESSENGINEOPTIONDOESNTEXIST          (BYTE)0xf6
#define ERROR_CMD_SEARCH_OPTION_DOESNT_EXIST            (BYTE)0xf5




#define ERROR_CMD_PAWNCOLLIDEDSTRAIGHT                  (BYTE)0xf4
#define ERROR_CMD_PAWNCOLLIDEDDIAGONALOREMPTYCELL       (BYTE)0xf3
#define ERROR_CMD_STARTINGCELLEMPTY                     (BYTE)0xf2
#define ERROR_CMD_NOTTHATCOLORSTURN                     (BYTE)0xf1
#define ERROR_CMD_MOVEINVALIDORBLOCKEDBYSAMECOLOR       (BYTE)0xf0
#define ERROR_CMD_CANNOTCASTLEKINGSIDE                  (BYTE)0xef
#define ERROR_CMD_CANNOTCASTLEQUEENSIDE                 (BYTE)0xee
#define ERROR_CMD_OWNKINGINCHECK                        (BYTE)0xed
#define ERROR_CMD_OTHERKINGINCHECKMATE                  (BYTE)0xec
#define ERROR_CMD_PAWNNOTALLOWEDNOTPROMOTIONMOVE        (BYTE)0xeb
#define ERROR_CMD_PIECETOPROMOTEISNOTPAWN               (BYTE)0xea
#define ERROR_CMD_PAWNNOTMOVINGTOENDOFBOARD             (BYTE)0xe9
#define ERROR_CMD_INVALIDPIECENAMETOPROMOTEINTO         (BYTE)0xe8
#define ERROR_CMD_CHESSENGINECREATEDNOMOVE              (BYTE)0xe7
#define ERROR_CMD_INVALIDMOVEFORMAT                     (BYTE)0xe6
#define ERROR_CMD_NOCHESSBOARDINFORMATIONFromCamera     (BYTE)0xe5
#define ERROR_CMD_PICKUPCELLEMPTY                       (BYTE)0xe4
#define ERROR_CMD_SYSTEMINPLAYCHESSSTATEMACHINE         (BYTE)0xe3