/**
  * @copyright Copyright (c) 2023, HiSilicon (Shanghai) Technologies Co., Ltd. All rights reserved.
  * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
  * following conditions are met:
  * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
  * disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
  * following disclaimer in the documentation and/or other materials provided with the distribution.
  * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote
  * products derived from this software without specific prior written permission.
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
  * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
  * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
  * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  * @file      app_command.c
  * @author    MCU Driver Team
  * @brief     command module driver
  * @details   The header file contains the following declaration:
  *             + Receive and parse the command input through the serial port.
  *             + implementation of serial port output function
  *             + Key Character Detection
  */
#include <stdio.h>
#include <string.h>
#include "common.h"
#include "command.h"
#include "console.h"
#include "cmd_common.h"
#include "cmd.h"
#include "securec.h"

typedef struct {
    char ch;
    char c;
} DirMapRes;

/**
 * @brief Command Input Keyword Initialization
 */
struct CmdInput {
    char buf[CMD_BUF_MAX];
    size_t cursor;
};

/**
 * @brief Initialize the command storage address.
 */
struct CmdRecord {
    char cmdLogList[CMD_NUM_MAX][CMD_BUF_MAX];
    signed char max;
    signed char addIdx;
    signed char cur;
};

/**
 * @brief Initialize the command sending variable.
 */
struct CmdCtx {
    char cmdBuf[CMD_BUF_MAX];
    const char *argv[ARGS_NUM_MAX];
    unsigned char dirKeyLen;
    char uartRxch; /* stores the characters */

    struct CmdInput inputCmd;
    struct CmdRecord cmdLog;
};

static struct CmdCtx g_cmdCtx = { 0 };
static struct CmdCtx *AppCmdGetCtx(void)
{
    /* Initialize the structure value */
    return &g_cmdCtx;
}

/**
 * @brief Get Previous Command
 * @param cmdLog: Commands from user
 * @retval : Parse the subscript or error return value of a character string.
 */
static char *GetPreCmd(struct CmdRecord *cmdLog)
{
    if (cmdLog->max == 0) { /* Failed to initialize the maximum value */
        return NULL;
    }

    if (cmdLog->max < CMD_NUM_MAX - 1) {
        if (cmdLog->cur == 0) {
            return NULL;
        }
        return cmdLog->cmdLogList[--cmdLog->cur];
    }

    if (cmdLog->cur == 0) {
        if (cmdLog->addIdx == cmdLog->max) {
            return NULL;
        }
        cmdLog->cur = cmdLog->max;
        return cmdLog->cmdLogList[cmdLog->cur];
    }

    if (cmdLog->addIdx == cmdLog->cur - 1) {
        return NULL;
    }
    return cmdLog->cmdLogList[--cmdLog->cur]; /* the pointer address is returned */
}

/**
 * @brief Read the next command
 * @param cmdLog: Commands from user
 * @retval : Parse the subscript or error return value of a character string.
 */
static char *GetNextCmd(struct CmdRecord *cmdLog)
{
    if (cmdLog == NULL || cmdLog->max == 0) { /* Failed to initialize the maximum value */
        return NULL;
    }

    if (cmdLog->max < CMD_NUM_MAX - 1) {
        if (cmdLog->cur == cmdLog->max) {
            return NULL;
        }
        return cmdLog->cmdLogList[++cmdLog->cur];
    }

    if (cmdLog->cur == cmdLog->max) {
        if (cmdLog->addIdx == 0) {
            return NULL;
        }
        cmdLog->cur = 0;
        return cmdLog->cmdLogList[cmdLog->cur];
    }

    if (cmdLog->addIdx == cmdLog->cur + 1) {
        return NULL;
    }
    return cmdLog->cmdLogList[++cmdLog->cur]; /* the pointer address is returned */
}

/**
 * @brief Delete End Identifier
 * @param inputcmd : Entered character string information.
 * @retval None.
 */
static void CmdDeleteTailChar(struct CmdInput *inputCmd)
{
    if (inputCmd == NULL || inputCmd->cursor == 0) {
        return;
    }
    /* Add a closing marker to a string */
    ConsolePutc(CTL_BACKSPACE);
    ConsolePutc(SPACE_KEY);
    ConsolePutc(CTL_BACKSPACE);
    inputCmd->buf[--(inputCmd->cursor)] = '\0';
}

/**
 * @brief Add a terminator at the end of a string
 * @param inputcmd : Entered character string information.
 * @retval None.
 */
static void CmdAddTailChar(struct CmdInput *inputCmd, char ch)
{
    if (inputCmd->cursor >= CMD_BUF_MAX - 1) {
        return;
    }
    inputCmd->buf[(inputCmd->cursor)++] = ch;
    /* Remove '\n' characters and add '\r\n' */
    ConsolePutc(ch);
}

/**
 * @brief Ignore the effects of key characters
 * @param ch : Characters contained in the command
 * @retval Indicates whether the implementation is successful.
 */
static unsigned char IgnoreCmdKey(char ch)
{
    /* end character and type character for the crt key */
    char ignoreKeys[] = {'\0', CTL_CH('a'), CTL_CH('b'), CTL_CH('e'), CTL_CH('f'), CTL_CH('x'),
                         CTL_CH('o'), CTL_CH('u')};

    for (unsigned char i = 0; i < sizeof(ignoreKeys); i++) {
        if (ch == ignoreKeys[i]) {
            return EXT_TRUE;
        }
    }
    return EXT_FALSE;
}

/**
 * @brief deleted Command Keys
 * @param ch : Characters contained in the command
 * @retval Indicates whether the implementation is successful.
 */
static unsigned char DeleteCmdKey(char ch)
{
    /* Backspace key delete key and other special key input */
    char deleteKeys[] = {DEL, DEL7, CTL_CH('h'), CTL_CH('d'), CTL_CH('k')};

    for (unsigned char i = 0; i < sizeof(deleteKeys); i++) {
        if (ch == deleteKeys[i]) {
            return EXT_TRUE;
        }
    }
    return EXT_FALSE;
}

/**
 * @brief Output Log Commands
 * @param ch : Characters contained in the command
 * @param cmdlog : Command log information
 * @param input : Entering command information
 * @retval None
 */
static void OutputLogCmd(struct CmdCtx *cmdCtx)
{
    char *cmd = NULL;

    cmd = (cmdCtx->uartRxch  == CTL_CH('p')) ? GetPreCmd(&(cmdCtx->cmdLog)) : GetNextCmd(&(cmdCtx->cmdLog));
    /* If the value is empty, direct returned */
    if (cmd == NULL) {
        return;
    }

    /* Clears the array of characters */
    while (cmdCtx->inputCmd.cursor) {
        CmdDeleteTailChar(&(cmdCtx->inputCmd));
    }
    if (strncpy_s(cmdCtx->inputCmd.buf, CMD_BUF_MAX, cmd, strlen(cmd)) != EXT_SUCCESS) {
        APP_CMD_ERR_PRINT("backup logcmd err\n");
        return;
    }
    /* Update pointer coordinates */
    cmdCtx->inputCmd.cursor = strlen(cmdCtx->inputCmd.buf);
    EXT_PRINT("%s", cmdCtx->inputCmd.buf);
}

/**
 * @brief Output log commands
 * @param cmd : Command string
 * @param cmdlog : Command log information
 * @retval Indicates whether the implementation is successful
 */
static int RecordCmd(const char *cmd, struct CmdRecord *cmdLog)
{
    /* not record uart switch cmd */
    if (strncmp(cmd, UART_SWITCH_CMD, strlen(UART_SWITCH_CMD)) == 0) {
        return EXT_SUCCESS;
    }

    /* clear buf and copy cmd to buf */
    memset_s(cmdLog->cmdLogList[cmdLog->addIdx], CMD_BUF_MAX, 0, CMD_BUF_MAX);
    if (strncpy_s(cmdLog->cmdLogList[cmdLog->addIdx], CMD_BUF_MAX, cmd, strlen(cmd)) != EXT_SUCCESS) {
        return EXT_FAILURE;
    }
    cmdLog->addIdx = (cmdLog->addIdx + 1) % CMD_NUM_MAX;
    cmdLog->max = (cmdLog->addIdx > cmdLog->max) ? cmdLog->addIdx : cmdLog->max;
    cmdLog->cur = cmdLog->addIdx;
    return EXT_SUCCESS;
}

/**
 * @brief Clear the command and initialize the address
 * @param inputCmd : Entering command information
 * @retval None
 */
static void CleanInputCmd(struct CmdCtx *cmdCtx)
{
    /* the address pointer points to the start address */
    cmdCtx->inputCmd.buf[0] = '\0';
    cmdCtx->inputCmd.cursor = 0;
    ConsolePuts("\n$ ");
}

/**
 * @brief Parse the arrow keys in the command.
 * @param ch : Characters entered
 * @param dirKeyLen : Arrow key flag
 * @retval end character or non-direction character entered
 */
static char CmdDirectionKey(char ch, unsigned char *dirKeyLen)
{
    char c = '\0';
    DirMapRes dirMap[5] = {
        /* Dir chd have 5 */
        {'D', CTL_CH('b')}, /* left key, convert to ctrl + b */
        {'C', CTL_CH('f')}, /* right key, convert to ctrl + c */
        {'H', CTL_CH('a')}, /* Home key, convert to ctrl + a */
        {'A', CTL_CH('p')}, /* up arrow, convert to ctrl + p */
        {'B', CTL_CH('n')}  /* down arrow, convert to ctrl + n */
    };

    if (*dirKeyLen == 0) {
        if (ch == DIR_KEY_HEAD) {
            *dirKeyLen = 1;
            return c;
        }
        return ch;
    }

    if (*dirKeyLen == 1) {
        *dirKeyLen = (ch == '[') ? 2 : 0; /* 2 is directionKeyLen */
        return c;
    }

    /* handle the third char sended by direction key */
    for (unsigned char i = 0; i < sizeof(dirMap) / sizeof(DirMapRes); i++) {
        if (ch == dirMap[i].ch) {
            c = dirMap[i].c;
            break;
        }
    }
    *dirKeyLen = 0;
    return c;
}

/**
 * @brief tabkey alignment implementation
 * @param res : Entered string
 * @param inputCmd : Entering command information
 * @retval None
 */
static void CompletesTabKey(const char *res, struct CmdInput *inputCmd)
{
    if (res == NULL || inputCmd == NULL) {
        APP_CMD_ERR_PRINT("param err\n");
        return;
    }
    /* Obtains the array length */
    size_t len = strlen(res);

    while (len > inputCmd->cursor && inputCmd->cursor < CMD_BUF_MAX) {
        /* Output Characters */
        ConsolePutc(res[inputCmd->cursor]);

        inputCmd->buf[inputCmd->cursor] = res[inputCmd->cursor];
        inputCmd->cursor++;
    }
}

/**
 * @brief tabkey alignment implementation
 * @param inputCmd : Entering command information
 * @retval None
 */
static void CmdTabKey(struct CmdCtx *cmdCtx)
{
    const char* res[MATCH_CMD_BUF_CNT] = { NULL };
    /* Numeric element initialization */
    unsigned char findCnt = 0;
    unsigned char cycle = 0;
    unsigned int tailId = 0;
    unsigned char searchFinish = EXT_TRUE;

    cmdCtx->inputCmd.buf[cmdCtx->inputCmd.cursor] = '\0';
    while (EXT_TRUE) {
    /* The printing is performed cyclically until the tabkey detection is complete */
        searchFinish = ExtCmdFindMatchCmd(cmdCtx->inputCmd.buf, res, MATCH_CMD_BUF_CNT, &findCnt, &tailId);
        cycle++;
        if (searchFinish && cycle ==1) {
            if (findCnt) {
                CompletesTabKey(res[0], &(cmdCtx->inputCmd));
            }
            break;
        }
		/* Print newline key after end */
        if (cycle == 1) {
            EXT_PRINT("\n");
        }
        /* cyclic print characters */
        for (unsigned char i = 0; i < findCnt; i++) {
            EXT_PRINT("%s ", res[i]);
        }
        EXT_PRINT("\n");
        if (searchFinish) {
            EXT_PRINT("$");
            EXT_PRINT("%s", cmdCtx->inputCmd.buf);
            break;
        }
        /* Clear the count value */
        findCnt = 0;
    }
}

/**
 * @brief tabkey alignment implementation
 * @param inputCmd : Entering command information
 * @param cmdBuf ；Command storage array
 * @param cmdLog : Address for storing printed log information
 * @retval None
 */
static void CmdEnterKey(struct CmdCtx *cmdCtx)
{
    if (cmdCtx->inputCmd.cursor == 0) {
        ConsolePuts("\n$ ");
        return;
    }

    if (memcpy_s(cmdCtx->cmdBuf, CMD_BUF_MAX, cmdCtx->inputCmd.buf, cmdCtx->inputCmd.cursor) != EXT_SUCCESS) {
        ConsolePuts("\n$ "); /* Add the end character */
        return;
    }
    cmdCtx->cmdBuf[cmdCtx->inputCmd.cursor] = '\0';

    if (RecordCmd(cmdCtx->cmdBuf, &(cmdCtx->cmdLog)) != EXT_SUCCESS) {
        ConsolePuts("\n$ "); /* Add the end character */
        return;
    }
    CleanInputCmd(cmdCtx);
}

/**
 * @brief Setting the log level
 * @param None
 * @retval Returns the value of the character's ASCII code
 */
static int CmdStrSetLevel(void)
{
    int ret;
    for (unsigned char i = 0; i < EXT_MODULE_BUTT; i++) {
        /* Setting the log level cyclically */
        ret = ExtDrvLogSetLogLevel(i, EXT_LOG_LEVEL_FATAL);
        if (ret != EXT_SUCCESS) {
            break;
        }
    }
    return ret;
}

typedef struct {
    unsigned int ulEventBit;
    void (*Func)(struct CmdCtx *cmdCtx);
} EventDoWithTable_t;
static const EventDoWithTable_t astDoWithTable[] = {
    { CTL_CH_C, CleanInputCmd},
    { TAB_KEY, CmdTabKey}, /* detrct the tab key */
    { ENTER_KEY1, CmdEnterKey}, /* detected '\r' */
    { ENTER_KEY2, CmdEnterKey}, /* detected '\n' */
    { CTL_CH_P, OutputLogCmd},
    { CTL_CH_N, OutputLogCmd}
};
/**
 * @brief Parse characters one by one
 * @param cmdCtx : string information
 * @retval Returns the value of the character's ASCII code
 */
static int GetCmdStr(struct CmdCtx *cmdCtx)
{
    int ret = EXT_SUCCESS;
    char c;
    /* Query the status of the serial port register */
    while ((cmdCtx->uartRxch = ConsoleGetc()) != 0) {
        if (cmdCtx->inputCmd.cursor >= CMD_BUF_MAX) {
            APP_CMD_ERR_PRINT("\ncmd overflow\n");
            /* Clear Character Cache */
            CleanInputCmd(cmdCtx);
            return EXT_FAILURE;
        }
        /* Get cmd direct key word */
        c = CmdDirectionKey(cmdCtx->uartRxch, &cmdCtx->dirKeyLen);
        if (IgnoreCmdKey(c)) {
            return ret;
        }
        /* Delete cmd key word */
        if (DeleteCmdKey(c)) {
            CmdDeleteTailChar(&cmdCtx->inputCmd);
            return ret;
        }
        /* Invoke the function drive table */
        for (unsigned int i = 0 ; i < (unsigned int)sizeof(astDoWithTable)/sizeof(astDoWithTable[0]); i ++) {
            if ((unsigned int)c == astDoWithTable[i].ulEventBit) {
                astDoWithTable[i].Func(cmdCtx);
                return ret;
            }
        }
        /* Setting the log level */
        if (c == CTL_CH('l')) {
            ret = CmdStrSetLevel();
            return ret;
        }
        /* Add key word to tail */
        CmdAddTailChar(&cmdCtx->inputCmd, c);
    }
    return ret;
}

/**
 * @brief encapsulation of Serial Port Transmission
 * @param None
 * @retval None
 */
void ExtAppCmdProcess(void)
{
    int ret;
    unsigned int argsNum;
    struct cmdRegisterTable *cmd;
    /* Initialize the structure */
    struct CmdCtx *cmdCtx = AppCmdGetCtx();
    ret = GetCmdStr(cmdCtx);
    if ((ret != EXT_SUCCESS) || (cmdCtx->cmdBuf[0] == '\0')) {
        return;
    }
    argsNum = CmdParserParam(cmdCtx->cmdBuf, cmdCtx->argv);
    if (argsNum == 0) {
        /* Clear the memory in the structure */
        (void)memset_s(&cmdCtx->inputCmd, sizeof(cmdCtx->inputCmd), 0, sizeof(cmdCtx->inputCmd));
        (void)memset_s(&cmdCtx->cmdBuf, sizeof(cmdCtx->cmdBuf), 0, sizeof(cmdCtx->cmdBuf));
        return;
    }
    cmd = ExtCmdFindCmd(cmdCtx->argv[0]);
    if (cmd == NULL || cmd->func == NULL) {
        /* Initialization Structure */
        (void)memset_s(&cmdCtx->inputCmd, sizeof(cmdCtx->inputCmd), 0, sizeof(cmdCtx->inputCmd));
        (void)memset_s(&cmdCtx->cmdBuf, sizeof(cmdCtx->cmdBuf), 0, sizeof(cmdCtx->cmdBuf));
        return;
    }
    cmd->func(argsNum, cmdCtx->argv);
    EXT_PRINT("\n$ ");
    /* Initialization Structure */
    (void)memset_s(&cmdCtx->inputCmd, sizeof(cmdCtx->inputCmd), 0, sizeof(cmdCtx->inputCmd));
    (void)memset_s(&cmdCtx->cmdBuf, sizeof(cmdCtx->cmdBuf), 0, sizeof(cmdCtx->cmdBuf));
}
