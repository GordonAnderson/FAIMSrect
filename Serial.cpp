/*
 * serial.c
 *
 * This file contains the code for the serial IO hardware including the ring buffer
 * code and the serial command processor.
 *
 * Developed by: Gordon Anderson
 */
#include "Arduino.h"
#include "string.h"
#include "Serial.h"
#include "Errors.h"
#include <Wire.h>
#include <SPI.h>
#include "Hardware.h"

extern ThreadController control;

Stream *serial = &Serial;
bool SerialMute = false;

#define MaxToken 20

char Token[MaxToken];
char Sarg1[MaxToken];
char Sarg2[MaxToken];
unsigned char Tptr;

int ErrorCode = 0;   // Last communication error that was logged

Ring_Buffer  RB;     // Receive ring buffer

// ACK only string, two options. Need a comma when in the echo mode
char *ACKonlyString1 = (char *)"\x06";
char *ACKonlyString2 = (char *)",\x06";
char *SelectedACKonlyString = ACKonlyString1;

bool echoMode = false;

Commands  CmdArray[] =   {
// General commands
  {"GVER",  CMDstr, 0, (char *)Version},                                  // Report version
  {"GERR",  CMDint, 0, (char *)&ErrorCode},                               // Report the last error code
  {"MUTE",  CMDfunctionStr, 1, (char *)Mute},                             // Turns on and off the serial response from the MIPS system
  {"ECHO",  CMDbool, 1, (char *)&echoMode},                               // Turns on and off the serial echo mode where the command is echoed to host, TRUE or FALSE
  {"DELAY", CMDfunction, 1, (char *)DelayCommand},                        // Generates a delay in milliseconds. This is used by the macro functions
                                                                          // to define delays in voltage ramp up etc.
  {"GCMDS",  CMDfunction, 0, (char *)GetCommands},                        // Send a list of all commands
  {"GNAME", CMDstr, 0, (char *)faims.Name},                               // Report system name
  {"SNAME", CMDstr, 1, (char *)faims.Name},                               // Set system name
  {"RESET",  CMDfunction, 0, (char *)Software_Reset},                     // System reboot
  {"SAVE",   CMDfunction, 0, (char *)SaveSettings},                       // Save settings
  {"RESTORE", CMDfunction, 0, (char *)RestoreSettings},                   // Restore settings
  {"FORMAT", CMDfunction, 0, (char *)FormatFLASH},                        // Format FLASH
  {"DEBUG", CMDfunction, 1, (char *)Debug},                               // Debug function, its function varies
  {"THREADS", CMDfunction, 0, (char *)ListThreads},                       // List all threads, there IDs, and there last runtimes
  {"STHRDENA", CMDfunctionStr, 2, (char *)SetThreadEnable},               // Set thread enable to true or false
  {"BLOAD",  CMDfunction, 0, (char *)bootLoader},                         // Start the boot loader
// FAIMS general commands
  {"SFMENA",  CMDbool, 1, (char *)&faims.Enable},                         // Faims enable, TRUE or FALSE
  {"GFMENA",  CMDbool, 0, (char *)&faims.Enable},
  {"SFREQ",  CMDfunction, 1, (char *)setFrequency},                       // FAIMS frequency, in Hz
  {"GFREQ",  CMDint, 0, (char *)&faims.Freq},
  {"SDUTY",  CMDfunctionStr, 1, (char *)setDuty},                         // FAIMS duty cycle, in percentage
  {"GDUTY",  CMDfloat, 0, (char *)&faims.Duty},
  {"SV",  CMDfunctionStr, 1, (char *)setVoltage},                         // FAIMS voltage, peak to peak voltage
  {"GV",  CMDfloat, 0, (char *)&faims.Voltage},
  {"SMAXI",  CMDfunctionStr, 1, (char *)setMaxI},                         // Set the power supplies max current limit
  {"GMAXI",  CMDfloat, 0, (char *)&faims.MaxI},
  {"SLOADC",  CMDfunctionStr, 1, (char *)setLoadC},                       // Set the load capacitance in pF
  {"GLOADC",  CMDfloat, 0, (char *)&faims.LoadC},
// CV and Bias parameters
  {"SREVCVB", CMDbool, 1, (char *)&revCVbias},                            // Reverse Bias and CV voltages on output connectors
  {"GREVCVB", CMDbool, 0, (char *)&revCVbias},
  {"SCV",     CMDfunctionStr, 1, (char *)setCV},                          // Set the CV value in volts
  {"GCV",     CMDfloat, 0, (char *)&faims.CV},
  {"GCVV",     CMDfloat, 0, (char *)&CVmon},
  {"SBIAS",   CMDfunctionStr, 1, (char *)setBias},                        // Set the Bias value in volts
  {"GBIAS",   CMDfloat, 0, (char *)&faims.Bias},
  {"GBIASV",   CMDfloat, 0, (char *)&BIASmon},
  {"SCVALT",  CMDfunctionStr, 1, (char *)setCValt},                       // Set the CV alternate value in volts
  {"GCVALT",  CMDfloat, 0, (char *)&faims.CValt},
  {"SALTENA", CMDbool, 1, (char *)&faims.EnableAlt},                      // Enables the alternate CV value to be applided on trigger
  {"GALTENA", CMDbool, 0, (char *)&faims.EnableAlt},
  {"SCVSTRT", CMDfunctionStr, 1, (char *)setCVstart},                     // Set the CV scan start voltage
  {"GCVSTRT", CMDfloat, 0, (char *)&faims.CVstart},
  {"SCVSTP", CMDfunctionStr, 1, (char *)setCVstop},                       // Set the CV scan stop voltage
  {"GCVSTP", CMDfloat, 0, (char *)&faims.CVstop},
  {"SDUR",  CMDfunction, 1, (char *)setDuration},                         // Set the scan duration in mSec
  {"GDUR",  CMDint, 0, (char *)&faims.Duration},
  {"SSCNTRG", CMDbool, 1, (char *)&faims.ScanExtTrig},                    // Enables external trigger to start a CV scan
  {"GSCNTRG", CMDbool, 0, (char *)&faims.ScanExtTrig},
  {"TOGGLE", CMDfunction, 0, (char *)startToggle},                        // Start CV toggle
  {"SCAN", CMDfunction, 0, (char *)startScan},                            // Start CV scan
  {"STOP", CMDfunction, 0, (char *)stopScan},                             // Stop CV scan and toggle
  {"ISSCN", CMDfunction, 0, (char *)isScanning},                          // Returns TRUE if scanning
// Monitor values
  {"GHV",     CMDfloat, 0, (char *)&HVmonV[0]},
  {"GHVAUX",  CMDfloat, 0, (char *)&HVmonV[1]},
  {"GHVI",    CMDfloat, 0, (char *)&HVmonI[0]},
  {"GHVIAUX", CMDfloat, 0, (char *)&HVmonI[1]},
  {"G12V", CMDfloat, 0, (char *)&supply},
  {"GPWR",    CMDfloat, 0, (char *)&power},
// Calibration functions
  {"CALCV",   CMDfunction, 0, (char *)CalibrateCV},
  {"CALBIAS", CMDfunction, 0, (char *)CalibrateBias},
  {"CALHV", CMDfunction, 0, (char *)CalibrateHV},
  {"CALHVAUX", CMDfunction, 0, (char *)CalibrateHVaux},
  {"CALHVI", CMDfunction, 0, (char *)CalibrateHVI},
  {"CALHVIAUX", CMDfunction, 0, (char *)CalibrateHVIaux},
// End of table marker
  {0},
};

// Sends a list of all commands
void GetCommands(void)
{
  int  i;

  SendACKonly;
  // Loop through the commands array and send all the command tokens
  for (i = 0; CmdArray[i].Cmd != 0; i++)
  {
    serial->println((char *)CmdArray[i].Cmd);
  }
}

// Delay command, delay is in millisecs
void DelayCommand(int dtime)
{
  delay(dtime);
  SendACK;
}

// Turns on and off responses from the MIPS system
void Mute(char *cmd)
{
  if (strcmp(cmd, "ON") == 0)
  {
    SerialMute = true;
    SendACK;
    return;
  }
  else if (strcmp(cmd, "OFF") == 0)
  {
    SerialMute = false;
    SendACK;
    return;
  }
  SetErrorCode(ERR_BADARG);
  SendNAK;
}

void SerialInit(void)
{
  Serial.begin(115200);
  RB_Init(&RB);
}

// This function builds a token string from the characters passed.
void Char2Token(char ch)
{
  Token[Tptr++] = ch;
  if (Tptr >= MaxToken) Tptr = MaxToken - 1;
}

// Get token from string object, delimited with comma
// Token are numbered 1 through n, returns empty string
// at end of tokens
String GetToken(String cmd, int TokenNum)
{
  int i, j, k;
  String Token;

  // If TokenNum is 0 or 1 then return the first token.
  // Return up to delimiter of the whole string.
  cmd.trim();
  if (TokenNum <= 1)
  {
    if ((i = cmd.indexOf(',')) == -1) return cmd;
    Token = cmd.substring(0, i);
    return Token;
  }
  // Find the requested token
  k = 0;
  for (i = 2; i <= TokenNum; i++)
  {
    if ((j = cmd.indexOf(',', k)) == -1) return "";
    k = j + 1;
  }
  Token = cmd.substring(k);
  Token.trim();
  if ((j = Token.indexOf(',')) == -1) return Token;
  Token = Token.substring(0, j);
  Token.trim();
  return Token;
}

// This function reads the serial input ring buffer and returns a pointer to a ascii token.
// Tokens are comma delimited. Commands strings end with a semicolon or a \n.
// The returned token pointer points to a standard C null terminated string.
// This function does not block. If there is nothing in the input buffer null is returned.
char *GetToken(bool ReturnComma)
{
  unsigned char ch;

  // Exit if the input buffer is empty
  while (1)
  {
    ch = RB_Next(&RB);
    if (ch == 0xFF) return NULL;
    if (Tptr >= MaxToken) Tptr = MaxToken - 1;
    if ((ch == '\n') || (ch == ';') || (ch == ':') || (ch == ',') || (ch == ']') || (ch == '['))
    {
      if (Tptr != 0) ch = 0;
      else
      {
        Char2Token(RB_Get(&RB));
        ch = 0;
      }
    }
    else RB_Get(&RB);
    // Place the character in the input buffer and advance pointer
    Char2Token(ch);
    if (ch == 0)
    {
      Tptr = 0;
      if ((Token[0] == ',') && !ReturnComma) return NULL;
      return Token;
    }
  }
}

char  *UserInput(char *message, void (*function)(void))
{
  char *tkn;
  
  // Flush the input ring buffer
  RB.Head=RB.Tail=RB.Count=RB.Commands=0;
  serial->print(message);
  // Wait for a line to be detected in the ring buffer
  while(RB.Commands == 0) 
  {
    ReadAllSerial();
    if(function != NULL) function();
  }
  // Read the token
  tkn = GetToken(true);
  if(tkn != NULL) if(tkn[0] == '\n') tkn = NULL;
  return tkn;
}

int   UserInputInt(char *message, void (*function)(void))
{
  char   *tkn;
  String arg;

  tkn = UserInput(message,function);
  // Flush the input ring buffer
  RB.Head=RB.Tail=RB.Count=RB.Commands=0;
  arg = tkn;
  return arg.toInt();
}

float UserInputFloat(char *message, void (*function)(void))
{
  char   *tkn;
  String arg;

  tkn = UserInput(message,function);
  // Flush the input ring buffer
  RB.Head=RB.Tail=RB.Count=RB.Commands=0;
  arg = tkn;
  return arg.toFloat();
}

double UserInputDouble(char *message, void (*function)(void))
{
  char   *tkn;
  String arg;
  double dval;

  tkn = UserInput(message,function);
  // Flush the input ring buffer
  RB.Head=RB.Tail=RB.Count=RB.Commands=0;
  arg = tkn;
  sscanf(arg.c_str(),"%lf", &dval);
  return dval;
}

void ExecuteCommand(Commands *cmd, int arg1, int arg2, char *args1, char *args2, float farg1)
{
  if (echoMode) SelectedACKonlyString = ACKonlyString2;
  else SelectedACKonlyString = ACKonlyString1;
  switch (cmd->Type)
  {
    case CMDbool:
      if (cmd->NumArgs == 0)   // If true then write the value
      {
        SendACKonly;
        if (!SerialMute)
        {
          if (*(cmd->pointers.boolPtr)) serial->println("TRUE");
          else serial->println("FALSE");
        }
      }
      if (cmd->NumArgs == 1)  // If true then read the value
      {
        if ((strcmp(args1, "TRUE") == 0) || (strcmp(args1, "FALSE") == 0))
        {
          if (strcmp(args1, "TRUE") == 0) *(cmd->pointers.boolPtr) = true;
          else *(cmd->pointers.boolPtr) = false;
          SendACK;
          break;
        }
        SetErrorCode(ERR_BADARG);
        SendNAK;
      }
      break;
    case CMDstr:
      if (cmd->NumArgs == 0)   // If true then write the value
      {
        SendACKonly;
        if (!SerialMute) serial->println(cmd->pointers.charPtr);
        break;
      }
      if (cmd->NumArgs == 1)  // If true then read the value
      {
        strcpy(cmd->pointers.charPtr, args1);
        SendACK;
        break;
      }
      break;
    case CMDint:
      if (cmd->NumArgs == 0)   // If true then write the value
      {
        SendACKonly;
        if (!SerialMute) serial->println(*(cmd->pointers.intPtr));
        break;
      }
      if (cmd->NumArgs == 1)
      {
        *(cmd->pointers.intPtr) = arg1;
        SendACK;
        break;
      }
    case CMDfloat:
      if (cmd->NumArgs == 0)   // If true then write the value
      {
        SendACKonly;
        if (!SerialMute) serial->println(*(cmd->pointers.floatPtr));
      }
      if (cmd->NumArgs == 1)
      {
        *(cmd->pointers.floatPtr) = farg1;
        SendACK;
        break;
      }
      break;
    case CMDfunction:
      if (cmd->NumArgs == 0) cmd->pointers.funcVoid();
      if (cmd->NumArgs == 1) cmd->pointers.func1int(arg1);
      if (cmd->NumArgs == 2) cmd->pointers.func2int(arg1, arg2);
      break;
    case CMDfunctionStr:
      if (cmd->NumArgs == 0) cmd->pointers.funcVoid();
      if (cmd->NumArgs == 1) cmd->pointers.func1str(args1);
      if (cmd->NumArgs == 2) cmd->pointers.func2str(args1, args2);
      break;
    case CMDfun2int1flt:
      if (cmd->NumArgs == 3) cmd->pointers.func2int1flt(arg1, arg2, farg1);
      break;
    default:
      SendNAK;
      break;
  }
}

// This function processes serial commands.
// This function does not block and returns -1 if there was nothing to do.
int ProcessCommand(void)
{
  String sToken;
  char   *Token, ch;
  int    i;
  static int   arg1, arg2;
  static float farg1;
  static enum  PCstates state;
  static int   CmdNum;
  static char  delimiter = 0;
  // The following variables are used for the long string reading mode
  static char *lstrptr = NULL;
  static int  lstrindex;
  static bool lstrmode = false;
  static int lstrmax;

  // Wait for line in ringbuffer
  if (state == PCargLine)
  {
    if (RB.Commands <= 0) return -1;
    CmdArray[CmdNum].pointers.funcVoid();
    state = PCcmd;
    return 0;
  }
  if (lstrmode)
  {
    ch = RB_Get(&RB);
    if (ch == 0xFF) return (-1);
    if (ch == ',') return (0);
    if (ch == '\r') return (0);
    if (ch == '\n')
    {
      lstrptr[lstrindex++] = 0;
      lstrmode = false;
      return (0);
    }
    lstrptr[lstrindex++] = ch;
    return (0);
  }
  Token = GetToken(false);
  if (Token == NULL) return (-1);
  if (Token[0] == 0) return (-1);
  if ((echoMode) && (!SerialMute))
  {
    if (strcmp(Token, "\n") != 0)
    {
      if (delimiter != 0) serial->write(delimiter);
      serial->print(Token);
    }
    if (strcmp(Token, "\n") == 0) delimiter = 0;
    else delimiter = ',';
  }
  switch (state)
  {
    case PCcmd:
      if (strcmp(Token, ";") == 0) break;
      if (strcmp(Token, "\n") == 0) break;
      CmdNum = -1;
      // Look for command in command table
      for (i = 0; CmdArray[i].Cmd != 0; i++) if (strcmp(Token, CmdArray[i].Cmd) == 0)
        {
          CmdNum = i;
          break;
        }
      if (CmdNum == -1)
      {
        SetErrorCode(ERR_BADCMD);
        SendNAK;
        break;
      }
      // If the type CMDfunctionLine then we will wait for a full line in the ring buffer
      // before we call the function. Function has not args and must pull tokens from ring buffer.
      if (CmdArray[i].Type == CMDfunctionLine)
      {
        state = PCargLine;
        break;
      }
      // If this is a long string read command type then init the vaiable to support saving the
      // string directly to the provided pointer and exit. This function must not block
      if (CmdArray[i].Type == CMDlongStr)
      {
        lstrptr = CmdArray[i].pointers.charPtr;
        lstrindex = 0;
        lstrmax = CmdArray[i].NumArgs;
        lstrmode = true;
        break;
      }
      if (CmdArray[i].NumArgs > 0) state = PCarg1;
      else state = PCend;
      break;
    case PCarg1:
      Sarg1[0] = 0;
      sToken = Token;
      sToken.trim();
      arg1 = sToken.toInt();
      farg1 = sToken.toFloat();
      strcpy(Sarg1, sToken.c_str());
      if (CmdArray[CmdNum].NumArgs > 1) state = PCarg2;
      else state = PCend;
      break;
    case PCarg2:
      Sarg2[0] = 0;
      sToken = Token;
      sToken.trim();
      arg2 = sToken.toInt();
      strcpy(Sarg2, sToken.c_str());
      if (CmdArray[CmdNum].NumArgs > 2) state = PCarg3;
      else state = PCend;
      break;
    case PCarg3:
      sToken = Token;
      sToken.trim();
      farg1 = sToken.toFloat();
      state = PCend;
      break;
    case PCend:
      if ((strcmp(Token, "\n") != 0) && (strcmp(Token, ";") != 0))
      {
        state = PCcmd;
        SendNAK;
        break;
      }
      i = CmdNum;
      CmdNum = -1;
      state = PCcmd;
      ExecuteCommand(&CmdArray[i], arg1, arg2, Sarg1, Sarg2, farg1);
      break;
    default:
      state = PCcmd;
      break;
  }
  return (0);
}

void RB_Init(Ring_Buffer *rb)
{
  rb->Head = 0;
  rb->Tail = 0;
  rb->Count = 0;
  rb->Commands = 0;
}

int RB_Size(Ring_Buffer *rb)
{
  return (rb->Count);
}

int RB_Commands(Ring_Buffer *rb)
{
  return (rb->Commands);
}

// Put character in ring buffer, return 0xFF if buffer is full and can't take a character.
// Return 0 if character is processed.
char RB_Put(Ring_Buffer *rb, char ch)
{
  if (rb->Count >= RB_BUF_SIZE) return (0xFF);
  rb->Buffer[rb->Tail] = ch;
  if (rb->Tail++ >= RB_BUF_SIZE - 1) rb->Tail = 0;
  rb->Count++;
  if (ch == ';') rb->Commands++;
  if (ch == '\r') rb->Commands++;
  if (ch == '\n') rb->Commands++;
  return (0);
}

// Get character from ring buffer, return NULL if empty.
char RB_Get(Ring_Buffer *rb)
{
  char ch;

  if (rb->Count == 0)
  {
    rb->Commands = 0;  // This has to be true if the buffer is empty...
    return (0xFF);
  }
  ch = rb->Buffer[rb->Head];
  if (rb->Head++ >= RB_BUF_SIZE - 1) rb->Head = 0;
  rb->Count--;
  // Map \r to \n
  //  if(Recording) MacroFile.write(ch);
  if (ch == '\r') ch = '\n';
  if (ch == ';') rb->Commands--;
  if (ch == '\n') rb->Commands--;
  if (rb->Commands < 0) rb->Commands = 0;
  return (ch);
}

// Return the next character in the ring buffer but do not remove it, return NULL if empty.
char RB_Next(Ring_Buffer *rb)
{
  char ch;

  if (rb->Count == 0) return (0xFF);
  ch = rb->Buffer[rb->Head];
  if (ch == '\r') ch = '\n';
  return (ch);
}

void PutCh(char ch)
{
  RB_Put(&RB, ch);
}

// This function lists all the current threads and there current state.
void ListThreads(void)
{
  int    i = 0;
  Thread *t;

  // Loop through all the threads and report the Thread name, ID, Interval, enabled state, and last run time
  SendACKonly;
  serial->println("Thread name,ID,Interval,Enabled,Run time");
  while (1)
  {
    t = control.get(i++);
    if (t == NULL) break;
    serial->print(t->getName()); serial->print(", ");
    serial->print(t->getID()); serial->print(", ");
    serial->print(t->getInterval()); serial->print(", ");
    if (t->enabled) serial->print("Enabled,");
    else serial->print("Disabled,");
    serial->println(t->runTimeMs());
  }
}

void SetThreadEnable(char *name, char *state)
{
  Thread *t;

  if ((strcmp(state, "TRUE") !=0) && (strcmp(state, "FALSE") != 0))
  {
    SetErrorCode(ERR_BADARG);
    SendNAK;
    return;
  }
  // Find thread by name
  t = control.get(name);
  if (t == NULL)
  {
    SetErrorCode(ERR_BADARG);
    SendNAK;
    return;
  }
  SendACKonly;
  if (strcmp(state, "TRUE") == 0) t->enabled = true;
  else t->enabled = false;
}
