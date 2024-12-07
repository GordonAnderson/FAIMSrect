#include "Hardware.h"
#include <Arduino.h>
#include "SPI.h"
#include <wiring_private.h>
#include <assert.h>

// Counts to value and value to count conversion functions.
// Overloaded for both DACchan and ADCchan structs.
float Counts2Value(int Counts, DACchan *DC)
{
  return (Counts - DC->b) / DC->m;
}

float Counts2Value(int Counts, ADCchan *ad)
{
  return (Counts - ad->b) / ad->m;
}

int Value2Counts(float Value, DACchan *DC)
{
  int counts;

  counts = (Value * DC->m) + DC->b;
  if (counts < 0) counts = 0;
  if (counts > 65535) counts = 65535;
  return (counts);
}

int Value2Counts(float Value, ADCchan *ac)
{
  int counts;

  counts = (Value * ac->m) + ac->b;
  if (counts < 0) counts = 0;
  if (counts > 65535) counts = 65535;
  return (counts);
}

// AD5629R IO routines. This is a 8 channel DAC module with a TWI
// interface. Two functions are provided, a generic 24 bit write
// function and a DAC channel write function.
// Turn on the internal reference: data = 0x00800001
// Override LDAC: data = 0x006000FF
// Write a channel: data = 0x002yvvvv
//    y = channel
//    vvvv = value
int AD5629write(uint8_t addr, uint32_t val)
{
  int iStat;

  Wire.beginTransmission(addr);
  Wire.write((val>>16) & 0xFF);
  Wire.write((val>>8) & 0xFF);
  Wire.write(val & 0xFF);
  iStat = Wire.endTransmission();
  return (iStat);
}

// AD5593 IO routines. This is a analog and digitial IO chip with
// a TWI interface. The following are low level read and write functions,
// the modules using this device are responsible for initalizing the chip.

// Write to AD5593
// Return 0 if no error else an error code is returned
int AD5593write(uint8_t addr, uint8_t pb, uint16_t val)
{
  int iStat;
  
  Wire.beginTransmission(addr);
  Wire.write(pb);
  Wire.write((val >> 8) & 0xFF);
  Wire.write(val & 0xFF);
  {
    //AtomicBlock< Atomic_RestoreState > a_Block;
    iStat = Wire.endTransmission();
  }
  return (iStat);
}

// Read from AD5593R
// returns -1 on any error
int AD5593readWord(uint8_t addr, uint8_t pb)
{
  int iStat;
  
  Wire.beginTransmission(addr);
  Wire.write(pb);
  {
    //AtomicBlock< Atomic_RestoreState > a_Block;
    iStat = Wire.endTransmission();
  }
  if(iStat != 0)
  {
    return (-1);
  }
  // Now read the data word
  int j = 0;
  Wire.requestFrom(addr, (uint8_t)2);
  j = Wire.read() << 8;
  j |= Wire.read();
  return(j);
}

int AD5593readADC(int8_t addr, int8_t chan)
{
   int iStat;
   
   // Select the ADC channel number
   if((iStat = AD5593write(addr, 0x02, (1 << chan))) != 0) return(-1);
   // Read the data and make sure the address is correct then left 
   // justify in 16 bits
   int i = AD5593readWord(addr, 0x40);
   if(((i >> 12) & 0x7) != chan) return(-1);
   i <<= 4;
   return(i & 0xFFFF);
}


int AD5593readADC(int8_t addr, int8_t chan, int8_t num)
{
  int i,j, val = 0;

  for (i = 0; i < num; i++) 
  {
    j = AD5593readADC(addr, chan);
    if(j == -1) return(-1);
    val += j;
  }
  return (val / num);
}

int AD5593writeDAC(int8_t addr, int8_t chan, int val)
{
   uint16_t  d;
   // convert 16 bit DAC value into the DAC data data reg format
   d = (val>>4) | (chan << 12) | 0x8000;
   return(AD5593write(addr, 0x10 | chan, d));
}

// This fuction will write to the MCP4725 12 bit single channel DAC.
// cmd = 0x40 to write to the DAC reg
// cmd = 0x60 to write to the DAC and the startup value memory
// value = 12 bit data left shiftted 4 bits
// Default address is 0x62
// Used in Twave module to set the HV ps (0 to 500) voltage
void MCP4725(uint8_t addr, uint8_t cmd, uint16_t value)
{
  Wire.beginTransmission(addr);
  Wire.write(cmd);
  Wire.write(value >> 8);
  Wire.write(value & 0xF0);
  Wire.endTransmission();
}

void Software_Reset(void)
{
  // in globals declaration section
  #define CPU_RESTART_ADDR (uint32_t *)0xE000ED0C
  #define CPU_RESTART_VAL 0x5FA0004
  #define CPU_RESTART (*CPU_RESTART_ADDR = CPU_RESTART_VAL);

  CPU_RESTART;
}

// Vector to the boot loader
void bootLoader(void)
{
  _reboot_Teensyduino_();
}
