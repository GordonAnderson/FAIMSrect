#ifndef Hardware_h
#define Hardware_h

typedef struct
{
  uint8_t Chan;              // ADC channel number 0 through max channels for chip
  float   m;                 // Calibration parameters to convert channel to engineering units
  float   b;                 // ADCcounts = m * value + b, value = (ADCcounts - b) / m
} ADCchan;

typedef struct
{
  uint8_t Chan;              // DAC channel number 0 through max channels for chip
  float   m;                 // Calibration parameters to convert engineering to DAC counts
  float   b;                 // DACcounts = m * value + b, value = (DACcounts - b) / m
} DACchan;


float Counts2Value(int  Counts, DACchan *DC);
float Counts2Value(int  Counts, ADCchan *AC);
int   Value2Counts(float Value, DACchan *DC);
int   Value2Counts(float Value, ADCchan *AC);

int AD5629write(uint8_t addr, uint32_t val);
int AD5593write(uint8_t addr, uint8_t pb, uint16_t val);
int AD5593readWord(uint8_t addr, uint8_t pb);
int AD5593readADC(int8_t addr, int8_t chan);
int AD5593readADC(int8_t addr, int8_t chan, int8_t num);
int AD5593writeDAC(int8_t addr, int8_t chan, int val);

void MCP4725(uint8_t addr, uint8_t cmd, uint16_t value);

void Software_Reset(void);
void bootLoader(void);
#endif
