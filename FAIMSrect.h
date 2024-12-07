#ifndef FAIMSrect_h
#define FAIMSrect_h
#include "Hardware.h"


#define MAXV        1500
#define MAXPWR      160
//#define MAXV        1000
//#define MAXPWR      120

#define FILTER   0.1

#define SIGNATURE  0xAA55A5A5

// Status monitoring parameters
#define FIXEDPWR  0
#define SYSTEMC   110
#define ERRPWR    4

// Teensy DIO pin assignments
#define   RFONLED     0
#define   STATUSLED   1
#define   TRIGOUT     2
#define   TRIGIN      3
#define   FAIMSPLUSE  4
#define   CONTROLPB   5
#define   ENABLE24    7
#define   BUZZER      11
#define   RFONLT      12
#define   ENABLEHV    16
#define   AUXENABLEHV 23

// Tennsy Analog input pin assignments
#define   HVMONV      A0
#define   HVMONI      A1
#define   RETURNI     A3
#define   MON12V      A6
#define   AUXHVMONV   A8
#define   AUXHVMONI   A7

// TWI addresses
#define   HVDACV      0x63
#define   HVDACI      0x62
#define   CVBIAS      0x11

// AD5593 channel assignments
#define   CVSET       0
#define   CVMON       1
#define   BIASSET     2
#define   BIASMON     3
#define   POSENA      4
#define   NEGENA      5

typedef struct
{
  bool          update;
  bool          Enable;
  float         Duty;
  int           Freq;
  float         Voltage;
  float         MaxI;
  float         CV;
  float         Bias;
} State;

typedef struct
{
  int16_t       Size;                   // This data structures size in bytes
  char          Name[20];               // Holds the module name, "FAIMSrec"
  int8_t        Rev;                    // Holds the board revision number
  // FAIMS control parameters
  bool          Enable;                 // Enables the waveform generation
  float         Duty;                   // Defines the waveform duty cycle
  int           Freq;                   // Defines the operating frequency
  float         Voltage;                // Defines peak to peak waveform voltage, V
  float         MaxI;                   // Defines the maximum current for each supply, mA
  float         CV;                     // CV, -600 to 600V
  float         Bias;                   // Bias, -600 to 600V
  float         MaxFreq;                // Defines maximum allowed frequency, 1MHz
  float         MaxPower;               // Defines maximum power limit, 100W
  // Protection parameters
  float         LoadC;                  // Load capacitance in pF
  // CV scanning and state parameters
  float         CValt;                  // Alternate CV state
  bool          EnableAlt;              // If true the alt CV value is applied when trigger in is high
  float         CVstart;                // Start scan voltage
  float         CVstop;                 // End scan voltage
  int           Duration;               // Scan time in mS
  bool          ScanExtTrig;            // If true then external trigger starts scan
  // Addresses and calibration structures
  int           HVdacV,HVdacI,CVBIASio; // TWI addresses
  DACchan       HVvoltCtrl;
  DACchan       HVcurCtrl;
  ADCchan       HVmonV;
  ADCchan       HVmonI;
  ADCchan       AUXHVmonV;
  ADCchan       AUXHVmonI;
  ADCchan       Mon12V;
  ADCchan       ReturnI;
  // CV Bias supply analog IO
  DACchan       CVsetCtrl;
  ADCchan       CVmon;
  DACchan       BIASsetCtrl;
  ADCchan       BIASmon;
  //
  unsigned int  Signature;              // Must be 0xAA55A5A5 for valid data
} FAIMSrec;

extern FAIMSrec  faims;

extern bool revCVbias;

// Monitored values
extern float HVmonV[2];
extern float HVmonI[2];
extern float CVmon,BIASmon;
extern float returnI;
extern float supply;
extern float power;

bool UpdateADCvalue(uint8_t TWIadd, ADCchan *achan, float *value, float filter = FILTER);

float dynamicFilter(float lastV, float newV, float thres = 10, float filter = 0.25);

void SaveSettings(void);
void RestoreSettings(void);
void FormatFLASH(void);
void Debug(int i);

void ReadAllSerial(void);
void ProcessSerial(bool scan=true);

void setFrequency(int freq);
void setDuty(char *duty);
void setVoltage(char *volt);
void setMaxI(char *val);
void setCV(char *val);
void setBias(char *val);
void setCValt(char *val);
void setCVstart(char *val);
void setCVstop(char *val);
void setLoadC(char *val);
void setDuration(int val);
void startToggle(void);
void startScan(void);
void stopScan(void);
void isScanning(void);

#endif
