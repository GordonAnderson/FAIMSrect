#include <Arduino.h>
#include "Wire.h"
//
//  FAIMSrec
//
//    Requires a teenys 4.0 microcontroller.
//    This application generates FAMIS rectangular waveforms with variable frequency, duty cycle
//    and voltage. This application also support CV scanning in both a linear ramp or toggle between
//    two seperate voltage setting.
//
//    Gordon Anderson
//    gaa@owt.com
//    509.628.6851
//
// Version history:
//  1.0
//      - Orginal version
//  1.1, Sept 26. 2023
//      - Added compile switch for 1500 volt model
//      - Added reverse CV and Bias command, note its not a savable parameter.
//      - Fixed several minor bugs
//      - Clamped several values at 0
//      - Cleaned up compile errors
//  1.2, Dec 26, 2023
//      - Increased max duration to 1000000mS
//  1.3, Jan 29, 2024
//      - Lowered the low freq limit to 10KHz
//  1.4, Oct 31, 2024
//      - Lowered the low freq limit to 1KHz
//
#include <SPI.h>
#include <Wire.h>

#include "FAIMSrect.h"
#include "Hardware.h"
#include "Errors.h"
#include "Serial.h"
#include "Button.h"
#include <arduino-timer.h>
#include <ThreadController.h>
#include <SerialBuffer.h>
#include <TimerOne.h>
#include <EEPROM.h>

const char   Version[] PROGMEM = "FAIMS RFWG version 1.4, Oct 31, 2024";
FAIMSrec     faims;

int eeAddress = 0;

SerialBuffer sb;

auto timer = timer_create_default();
bool triggerEnable = false;

bool revCVbias = false;

Button functionPin;

State sdata;
// Monitored values
float HVmonV[2];
float HVmonI[2];
float CVmon,BIASmon;
float returnI;
float supply;
float power;

// Scanning variables
#define SCANINTERVAL 2
bool  CVstate = false;
bool  scanning = false;
bool  toggling = false;
bool  triggerScan = false;
bool  scanAbort = false;
unsigned int startTime;
float savedCV;

// ThreadController that will control all threads
ThreadController control = ThreadController();
//Threads
Thread SystemThread = Thread();

FAIMSrec Rev_1_faims = {
  sizeof(FAIMSrec),
  "FAIMSrec",
  1,
  false,
  33,
  750000,
  200,
  50,
  0,0,                          // CV and Bias
  1000000,                      // Max frequency
  MAXPWR,                       // Max power
  0,                            // Load capacitance in pF
  0,false,                      // Alt CV and alt CV enable
  0,100,                        // CV start and stop, for scan
  1000,false,                   // Scan duration in mS and external scan start trigger enable
  0x63,0x62,0x11,
#if MAXV <= 1000
  {0,49.28,-103.05},                // HV voltage DAC
#else
  {0,22.11,291.93},                 // HV voltage DAC
#endif
  {0,799.77,-78.64},                // HV current limit DAC
#if MAXV <= 1000
  {HVMONV | 0x80,52.04,-914.08},    // HV voltage monitor ADC, processor channel
  {HVMONI | 0x80,780,0},            // HV current monitor ADC, processor channel
  {AUXHVMONV | 0x80,52.04,-914.08}, // Aux HV voltage monitor ADC, processor channel
  {AUXHVMONI | 0x80,799.21,-5.50},  // Aux HV current monitor ADC, processor channel
#else
  {HVMONV | 0x80,10.13,77.92},          // HV voltage monitor ADC, processor channel
  {HVMONI | 0x80,-210.85,24613.16},     // HV current monitor ADC, processor channel
  {AUXHVMONV | 0x80,10.13,77.92},       // Aux HV voltage monitor ADC, processor channel
  {AUXHVMONI | 0x80,-210.85,24613.16},  // Aux HV current monitor ADC, processor channel
#endif
  {MON12V | 0x80,5460,0},           // 12 volt power supply monitor
  {RETURNI | 0x80,1092,0},          // FAIMS return current monitor
  // CV and BIAS channels
  {CVSET,-31.93,32841.83},          // CV voltage select DAC
  {CVMON,25.73,32733.91},           // CV voltage monitor ADC
  {BIASSET,-31.93,32871.26},        // BIAS voltage select DAC
  {BIASMON,25.78,32742.52},         // BIAS voltage monitor ADC
  SIGNATURE
};

void AD5593init(int8_t addr)
{
  // General purpose configuration, set range for 2.5 volts
  AD5593write(addr, 3, 0x0100);
  // Set internal reference
  AD5593write(addr, 11, 0x0200);
  // Set LDAC mode
  AD5593write(addr, 7, 0x0000);
  // Set DAC outputs channels
  AD5593write(addr, 5, 0x0005);
  // Set ADC input channels
  AD5593write(addr, 4, 0x000A);
  // Set digital output channels
  AD5593write(addr, 8, 0x0030);
  // Turn off all pulldowns
  AD5593write(addr, 6, 0x0000);

  // Init all DACs and DO
  AD5593writeDAC(addr, 0, 32768);
  AD5593writeDAC(addr, 2, 32768);
  AD5593write(addr, 9, 0);
}

//
// Dymamic IIR filter function. Desiged to allow rapid updates for large changes
// and filter lower amplitude noise
//
// Procedure:
//  - Calculate the percentage change
//  - If percentage is over threshhold apply filter value
//  - If percentage is below threshold calculate filter value
//    based on error, filter = change% / thres% * filter
float dynamicFilter(float lastV, float newV, float thres, float filter)
{
  if(lastV == 0) return lastV * (1-filter) + newV * filter;
  float e = abs(lastV - newV) / abs(lastV) * 100;
  if(e < thres) filter = filter * e / thres;
  return (lastV * (1-filter) + newV * filter);
}

// Read the ADC channel from the AD5593, if the channel number bit 7 is set then this is
// a processor ADC input pin
bool UpdateADCvalue(uint8_t TWIadd, ADCchan *achan, float *value, float filter)
{
  int   val;
  float fval;

  if(revCVbias)
  {
    if(achan == &faims.CVmon) achan = &faims.BIASmon;
    else if(achan == &faims.BIASmon) achan = &faims.CVmon;
  }
  if((achan->Chan & 0x80) != 0)
  {
    // Here if this is a processor ADC pin
    val = 0;
    for(int i=0;i<64;i++) 
    {
      val += analogRead(achan->Chan & 0x7F);
      delayMicroseconds(1);
    }
    val /= 4;
    //val = analogRead(achan->Chan & 0x7F) << 4;
    fval = Counts2Value(val,achan);
    if(*value == -1) *value = fval;
    else *value = filter * fval + (1 - filter) * *value;
    return true;    
  }
  if((val = AD5593readADC(TWIadd, achan->Chan,10)) != -1)
  {
    fval = Counts2Value(val,achan);
    if(*value == -1) *value = fval;
//    else *value = dynamicFilter(*value, fval);
    else *value = filter * fval + (1 - filter) * *value;
    return true;
  }
  return false;
}

bool UpdateDACvalue(uint8_t TWIadd, DACchan *dchan, float *value, float *svalue, bool update)
{
  if(revCVbias)
  {
    if(dchan == &faims.CVsetCtrl) dchan = &faims.BIASsetCtrl;
    else if(dchan == &faims.BIASsetCtrl) dchan = &faims.CVsetCtrl;
  }
  if((update) || (*value != *svalue))
  {
    if((TWIadd & 0x60) == 0x60) MCP4725(TWIadd,0x60,Value2Counts(*value,dchan));
    else AD5593writeDAC(TWIadd, dchan->Chan, Value2Counts(*value,dchan));
    *svalue = *value;
    return true;
  }
  return false;
}

bool buzzerOFF(void *) { digitalWrite(BUZZER,LOW); return true;}
void systemMonitor(void)
{
  if(!faims.Enable) return;
  // Test max power limit
  if(power > faims.MaxPower)
  {
    // Disable system, turn on yellow LED and signal using buzzer
    faims.Enable = false;
    digitalWrite(STATUSLED,LOW);
    digitalWrite(BUZZER,HIGH);
    timer.in(500,buzzerOFF);
  }
  // Calculate the expected power using loadC 
  float ePwr = FIXEDPWR + HVmonV[0] * HVmonV[0] * SYSTEMC * 1e-12 * faims.Freq;
  ePwr += HVmonV[0] * HVmonV[0] * faims.LoadC * 1e-12 * faims.Freq;
  if((power - ePwr) > ERRPWR)
  {
    // Disable system, turn on yellow LED and signal using buzzer
    faims.Enable = false;
    digitalWrite(STATUSLED,LOW);
    digitalWrite(BUZZER,HIGH);
    timer.in(500,buzzerOFF);    
  }
}

void Update()
{
  // Set all the output values that have changed or is flaged for global update
  if((sdata.Enable != faims.Enable) || sdata.update)
  {
    if(faims.Enable)
    {
      digitalWrite(STATUSLED,HIGH);
      triggerEnable = true;
    }
    else
    {
      // Here if not enable, turn offon the HV supplies and the CV/BIAS supplies
      digitalWrite(ENABLEHV,LOW);
      digitalWrite(AUXENABLEHV,LOW);
      AD5593write(faims.CVBIASio, 0x09, (1 << POSENA) | (1 << NEGENA));
      // Turn on RF on led and light
      digitalWrite(RFONLT,LOW);
      digitalWrite(RFONLED,HIGH);
    }
    sdata.Enable = faims.Enable;
  }
  if((sdata.Duty != faims.Duty) || sdata.update) { analogWrite(FAIMSPLUSE, (int)((100 - faims.Duty) * 1024 / 100)); sdata.Duty = faims.Duty; }
  if((sdata.Freq != faims.Freq) || sdata.update) { analogWriteFrequency(FAIMSPLUSE, faims.Freq); sdata.Freq = faims.Freq; }
  if((sdata.Voltage != faims.Voltage) || sdata.update) UpdateDACvalue(faims.HVdacV, &faims.HVvoltCtrl, &faims.Voltage, &sdata.Voltage, sdata.update);
  if((sdata.MaxI != faims.MaxI) || sdata.update) UpdateDACvalue(faims.HVdacI, &faims.HVcurCtrl, &faims.MaxI, &sdata.MaxI, sdata.update);
  if((sdata.CV != faims.CV) || sdata.update) UpdateDACvalue(faims.CVBIASio, &faims.CVsetCtrl, &faims.CV, &sdata.CV, sdata.update);
  if((sdata.Bias != faims.Bias) || sdata.update) UpdateDACvalue(faims.CVBIASio, &faims.BIASsetCtrl, &faims.Bias, &sdata.Bias, sdata.update);
  sdata.update = false;
  // Read all the monitor values
  UpdateADCvalue(0, &faims.HVmonV, &HVmonV[0]);    if(HVmonV[0] < 0) HVmonV[0] = 0;
  UpdateADCvalue(0, &faims.AUXHVmonV, &HVmonV[1]); if(HVmonV[1] < 0) HVmonV[1] = 0;
  UpdateADCvalue(0, &faims.HVmonI, &HVmonI[0]);    if(HVmonI[0] < 0) HVmonI[0] = 0;
  UpdateADCvalue(0, &faims.AUXHVmonI, &HVmonI[1]); if(HVmonI[1] < 0) HVmonI[1] = 0;
  UpdateADCvalue(faims.CVBIASio, &faims.CVmon, &CVmon);
  UpdateADCvalue(faims.CVBIASio, &faims.BIASmon, &BIASmon);
  UpdateADCvalue(0, &faims.ReturnI, &returnI);
  UpdateADCvalue(0, &faims.Mon12V, &supply);
  power = (HVmonV[0] * HVmonI[0]/1000.0) + (HVmonV[1] * HVmonI[1]/1000.0);
  systemMonitor();
}

bool enableWarning(void *)
{
  static int count = 0;

  if(count++ < 7)
  {
    if(count & 1) digitalWrite(BUZZER,LOW);
    else digitalWrite(BUZZER,HIGH);
    return true;
  }
  //serial->println(count);
  if(count == 8)
  {
    if(faims.Enable)
    {
      // Here if enable, turn on the HV supplies and the CV/BIAS supplies
      digitalWrite(ENABLEHV,HIGH);
      digitalWrite(AUXENABLEHV,HIGH);
      // Logic low turns on the supplies
      AD5593write(faims.CVBIASio, 0x09, 0);
      // Turn on RF on led and light
      digitalWrite(RFONLT,HIGH);
      digitalWrite(RFONLED,LOW);   
    }
  }
  count = 10;
  if(triggerEnable==true) 
  {
    count=0;
    triggerEnable = false;
  }
  return true;
}

void setup() 
{
  // Disable the 24V supply
  pinMode(ENABLE24,OUTPUT);
  digitalWrite(ENABLE24,LOW);
  // Disable HV supplies
  pinMode(ENABLEHV,OUTPUT);
  digitalWrite(ENABLEHV,LOW);
  pinMode(AUXENABLEHV,OUTPUT);
  digitalWrite(AUXENABLEHV,LOW);
  // Turn off the buzzer and LEDs
  pinMode(BUZZER,OUTPUT);
  digitalWrite(BUZZER,LOW);
  pinMode(RFONLT,OUTPUT);
  digitalWrite(RFONLT,LOW);
  pinMode(RFONLED,OUTPUT);
  digitalWrite(RFONLED,HIGH);
  pinMode(STATUSLED,OUTPUT);
  digitalWrite(STATUSLED,HIGH);
  // Set trigger out low
  pinMode(TRIGOUT,OUTPUT);
  digitalWrite(TRIGOUT,HIGH);
  // Read the flash config contents and test the signature
  faims = Rev_1_faims;
  Restore();
  // Init serial communications
  SerialInit();
  // Init I2C interface
  Wire.begin();
  Wire.setClock(100000);
  // Init the AD5593
  AD5593init(faims.CVBIASio);
  // Set analog resolutions
  analogWriteResolution(10);
  analogReadResolution(12);
  
  // Enable the 24V supply
  pinMode(ENABLE24,OUTPUT);
  digitalWrite(ENABLE24,HIGH);

  // Configure Threads
  SystemThread.setName((char *)"Update");
  SystemThread.onRun(Update);
  SystemThread.setInterval(100);
  // Add thread to the controller
  control.add(&SystemThread);

  functionPin.begin(CONTROLPB);

  timer.cancel();
  timer.every(250, enableWarning);

  sdata.update = true;
  Update();
  for(int i=0;i<100;i++) functionPin.released();
}

void ReadAllSerial(void)
{
  // Put serial received characters in the input ring buffer
  if (Serial.available() > 0)
  {
    serial = &Serial;
    PutCh(Serial.read());
  }
}

// This function process all the serial IO and commands
void ProcessSerial(bool scan)
{
  // Put serial received characters in the input ring buffer
  if (Serial1.available() > 0)
  {
    serial = &Serial1;
    PutCh(Serial1.read());
  }
  // Put serial received characters in the input ring buffer
  if (Serial.available() > 0)
  {
    serial = &Serial;
    PutCh(Serial.read());
  }
  if (!scan) return;
  // If there is a command in the input ring buffer, process it!
  if (RB_Commands(&RB) > 0) while (ProcessCommand() == 0); // Process until flag that there is nothing to do
}

void loop() 
{
  ProcessSerial();
  control.run();
  timer.tick();
  if(functionPin.released())
  {
    faims.Enable = !faims.Enable;
  }
  if(scanning && triggerScan)
  {
    triggerScan = false;
    startTime = millis();
    UpdateDACvalue(faims.CVBIASio, &faims.CVsetCtrl, &faims.CV, &sdata.CV, true);
    timer.every(SCANINTERVAL,scanCV);    
  }
  if(toggling && faims.EnableAlt)
  {
    if((digitalRead(TRIGIN) == HIGH) || scanAbort)
    {
      if(faims.CV != savedCV)
      {
        faims.CV = savedCV;
        UpdateDACvalue(faims.CVBIASio, &faims.CVsetCtrl, &faims.CV, &sdata.CV, true);
      }
    }
    else
    {
      if(faims.CV != faims.CValt)
      {
        faims.CV = faims.CValt;
        UpdateDACvalue(faims.CVBIASio, &faims.CVsetCtrl, &faims.CV, &sdata.CV, true);
      }      
    }
    if(scanAbort)
    {
      toggling = false;
      scanAbort = false;
    }
  }
  // Test 12V power supply, if its below 6 volts then shutdown and wait for 
  // voltage to return then restart
  if(supply < 8.0)
  {
    faims.Enable = false;
    while(supply < 8.0)
    {
      digitalWrite(ENABLE24,LOW);
      digitalWrite(ENABLEHV,LOW);
      digitalWrite(AUXENABLEHV,LOW);
      digitalWrite(BUZZER,LOW);
      digitalWrite(RFONLT,LOW);
      digitalWrite(RFONLED,HIGH);
      digitalWrite(STATUSLED,HIGH);
      delay(50);
      Update();
    }
    setup();
  }
}

// Scanning functions

// This is a timer function called at the duration interval. This function toggles CV
// between its defined value and CValt until scanning is aborted.
bool toggleCV(void *)
{
  if(!toggling) return false;
  if(CVstate) faims.CV = savedCV;
  else faims.CV = faims.CValt;
  CVstate = !CVstate;
  if(scanAbort) faims.CV = savedCV;
  UpdateDACvalue(faims.CVBIASio, &faims.CVsetCtrl, &faims.CV, &sdata.CV, true);
  if(!scanAbort) return true;
  scanAbort = false;
  toggling = false;
  return false;
}

bool scanCV(void *)
{
  if(!scanning) return false;
  if((scanAbort) || ((millis() - startTime) > (unsigned int)faims.Duration))
  {
    scanAbort = scanning = false;
    faims.CV = savedCV;
    UpdateDACvalue(faims.CVBIASio, &faims.CVsetCtrl, &faims.CV, &sdata.CV, true);
    return false;
  }
  // Calculate CV at the current time.
  faims.CV = faims.CVstart + (faims.CVstop - faims.CVstart) * (float)(millis() - startTime) / (float)faims.Duration;
  UpdateDACvalue(faims.CVBIASio, &faims.CVsetCtrl, &faims.CV, &sdata.CV, true);
  return true;
}

void trigScanISR(void)
{
  if(scanning) triggerScan=true;
}

//
// Host commands
//
void SaveSettings(void)
{
  faims.Signature = SIGNATURE;
  EEPROM.put(eeAddress, faims);
  SendACK;
}

bool Restore(void)
{
  static FAIMSrec fm;

  // Read the flash config contents and test the signature
  EEPROM.get(eeAddress, fm);
  if(fm.Signature == SIGNATURE) 
  {
    faims = fm;
    return true;
  }
  return false;
}

void RestoreSettings(void)
{
  if(Restore()) 
  {
    SendACK; 
  }
  else
  {
    SetErrorCode(ERR_EEPROMWRITE);
    SendNAK;
    return;
  }
}

void FormatFLASH(void)
{
  EEPROM.put(eeAddress, Rev_1_faims);  
  SendACK;
}

bool setVariable(float *v,char *value, float LL, float UL)
{
  float d;

  sscanf(value,"%f",&d);
  if((d<LL)||(d>UL)) 
  {
    SetErrorCode(ERR_BADARG); 
    SendNAK;
    return false;
  }
  SendACK;
  *v = d;
  return true;
}

bool setVariable(int *v,char *value, int LL, int UL)
{
  int d;

  sscanf(value,"%d",&d);
  if((d<LL)||(d>UL)) 
  {
    SetErrorCode(ERR_BADARG); 
    SendNAK;
    return false;
  }
  SendACK;
  *v = d;
  return true;
}

bool setVariable(int *v,int *value, int LL, int UL)
{
  int d;

  d = *value;
  if((d<LL)||(d>UL)) 
  {
    SetErrorCode(ERR_BADARG); 
    SendNAK;
    return false;
  }
  SendACK;
  *v = d;
  return true;
}

void setFrequency(int freq) {setVariable(&faims.Freq,&freq,1000,1000000);}
void setDuty(char *duty) {setVariable(&faims.Duty,duty,5,95);}
void setVoltage(char *volt) {setVariable(&faims.Voltage,volt,0,MAXV);}
void setMaxI(char *val) {setVariable(&faims.MaxI,val,0,60);}
void setCV(char *val) {setVariable(&faims.CV,val,-400,400);}
void setBias(char *val) {setVariable(&faims.Bias,val,-400,400);}
void setCValt(char *val) {setVariable(&faims.CValt,val,-400,400);}
void setCVstart(char *val) {setVariable(&faims.CVstart,val,-400,400);}
void setCVstop(char *val) {setVariable(&faims.CVstop,val,-400,400);}
void setDuration(int val) {setVariable(&faims.Duration,&val,10,10000000);}
void setLoadC(char *val) {setVariable(&faims.LoadC,val,0,50);}

// This function start the CV toggle function. The CV is toggeled between The CV
// value and CValt. The duration values defines the number of mS at each value.
void startToggle(void) 
{
  SendACK;
  if((scanning)||(toggling)) return;
  toggling = true;
  scanAbort = false;
  CVstate = true;
  savedCV = faims.CV;
  timer.every(faims.Duration,toggleCV);
}

void startScan(void) 
{
  SendACK;
  if((scanning)||(toggling)) return;
  triggerScan = false;
  scanning = true;
  scanAbort = false;
  savedCV = faims.CV;
  faims.CV = faims.CVstart;
  detachInterrupt(TRIGIN);
  if(!faims.ScanExtTrig) triggerScan = true;
  else attachInterrupt(TRIGIN,trigScanISR,RISING);
}

void stopScan(void) 
{
  SendACK;
  if((scanning)||(toggling)) scanAbort = true;
  if(scanning) triggerScan = true;
}

void isScanning(void)
{
  SendACKonly;
  if(SerialMute) return;
  if((scanning)||(toggling)) serial->println("TRUE");
  else serial->println("FALSE");
}

void Debug(int Mode)
{
}
