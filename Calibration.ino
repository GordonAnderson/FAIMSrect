// - Calibration functions
//    - CV and Bias
//    - HV voltage and readback
//    - HV current monitor

void CalibrateLoop(void)
{
  ProcessSerial(false);
  control.run();
}

int CalibratePoint(uint8_t TWIadd, DACchan *dacchan, ADCchan *adcchan, float *V)
{
  char   *Token;
  String sToken;
  int    val;

  // Set value and ask for user to enter actual value read
  if(dacchan !=NULL) 
  {
    if((TWIadd & 0x60) == 0x60) MCP4725(TWIadd,0x60,Value2Counts(*V,dacchan));
    else AD5593writeDAC(TWIadd, dacchan->Chan, Value2Counts(*V,dacchan));
  }
  serial->print("Enter actual value: ");
  while((Token = GetToken(true)) == NULL) CalibrateLoop();
  sToken = Token;
  serial->println(Token);
  *V = sToken.toFloat(); 
  while((Token = GetToken(true)) != NULL) CalibrateLoop(); 
  if(adcchan != NULL) 
  {
    if((adcchan->Chan & 0x80) == 0) return AD5593readADC(TWIadd, adcchan->Chan, 10);
    val = 0; 
    for(int i=0;i<64;i++) 
    {
      val += analogRead(adcchan->Chan & 0x7F);
      delayMicroseconds(1);
    }
    return val/4;
  }
  return 0; 
}

// This function is used to calibrate ADC/DAC AD5592 channels. 
void Calibrate(uint8_t TWIadd, DACchan *dacchan, ADCchan *adcchan, float V1, float V2)
{
  float  val1,val2,m,b;
  int    adcV1, adcV2;
  int    dacV1, dacV2;

  serial->println("Enter values when prompted.");
  // Set to first voltage and ask for user to enter actual voltage
  val1 = V1;
  adcV1 = CalibratePoint(TWIadd, dacchan, adcchan, &val1);
  // Set to second voltage and ask for user to enter actual voltage
  val2 = V2;
  adcV2 = CalibratePoint(TWIadd, dacchan, adcchan, &val2);
  // Calculate calibration parameters and apply
  dacV1 = Value2Counts(V1, dacchan);
  dacV2 = Value2Counts(V2, dacchan);
  m = (float)(dacV2-dacV1) / (val2-val1);
  b = (float)dacV1 - val1 * m;
  serial->println("DAC channel calibration parameters.");
  serial->print("m = ");
  serial->println(m);
  serial->print("b = ");
  serial->println(b);
  dacchan->m = m;
  dacchan->b = b;
  if(adcchan == NULL) return;
  m = (float)(adcV2-adcV1) / (val2-val1);
  b = (float)adcV1 - val1 * m;
  serial->println("ADC channel calibration parameters.");
  serial->print("m = ");
  serial->println(m);
  serial->print("b = ");
  serial->println(b);
  adcchan->m = m;
  adcchan->b = b;
}

void CalibrateCV(void)
{
  serial->println("Calibrate CV output, monitor with a voltmeter.");
  Calibrate(CVBIAS, &faims.CVsetCtrl, &faims.CVmon, 0.0, 300);
  AD5593writeDAC(CVBIAS, faims.CVsetCtrl.Chan, Value2Counts(faims.CV,&faims.CVsetCtrl));
}

void CalibrateBias(void)
{
  serial->println("Calibrate Bias output, monitor with a voltmeter.");
  Calibrate(CVBIAS, &faims.BIASsetCtrl, &faims.BIASmon, 0.0, 300);
  AD5593writeDAC(CVBIAS, faims.BIASsetCtrl.Chan, Value2Counts(faims.CV,&faims.BIASsetCtrl));
}

void CalibrateHV(void)
{
  serial->println("Calibrate HV output, monitor with a voltmeter.");
  Calibrate(HVDACV, &faims.HVvoltCtrl, &faims.HVmonV, 200, 750);
  UpdateDACvalue(0, &faims.HVvoltCtrl, &faims.Voltage, &sdata.Voltage, true);
}

void CalibrateHVaux(void)
{
  serial->println("Calibrate HV aux supply output, monitor with a voltmeter.");
  Calibrate(HVDACV, &faims.HVvoltCtrl, &faims.AUXHVmonV, 200, 750);
  UpdateDACvalue(0, &faims.HVvoltCtrl, &faims.Voltage, &sdata.Voltage, true);  
}

void CalibrateHVI(void)
{
  int    adcV1, adcV2;
  float  V1,V2;
  float  V = faims.Voltage;
  
  serial->println("Calibrate HV supply current, monitor with a voltmeter.");
  faims.Voltage = 200;
  UpdateDACvalue(HVDACV, &faims.HVvoltCtrl, &faims.Voltage, &sdata.Voltage, true);
  adcV1 = CalibratePoint(0, NULL, &faims.HVmonI, &V1);
  faims.Voltage = 750;
  UpdateDACvalue(HVDACV, &faims.HVvoltCtrl, &faims.Voltage, &sdata.Voltage, true);
  adcV2 = CalibratePoint(0, NULL, &faims.HVmonI, &V2);
  float m = (float)(adcV2-adcV1) / (V2-V1);
  float b = (float)adcV1 - V1 * m;
  serial->println("ADC channel calibration parameters.");
  serial->print("m = ");
  serial->println(m);
  serial->print("b = ");
  serial->println(b);
  faims.HVmonI.m = m;
  faims.HVmonI.b = b;
  faims.Voltage = V;
  UpdateDACvalue(HVDACV, &faims.HVvoltCtrl, &faims.Voltage, &sdata.Voltage, true);
}

void CalibrateHVIaux(void)
{
  int    adcV1, adcV2;
  float  V1,V2;
  float  V = faims.Voltage;
  
  serial->println("Calibrate HV aux supply current, monitor with a voltmeter.");
  faims.Voltage = 200;
  UpdateDACvalue(HVDACV, &faims.HVvoltCtrl, &faims.Voltage, &sdata.Voltage, true);
  adcV1 = CalibratePoint(0, NULL, &faims.AUXHVmonI, &V1);
  faims.Voltage = 750;
  UpdateDACvalue(HVDACV, &faims.HVvoltCtrl, &faims.Voltage, &sdata.Voltage, true);
  adcV2 = CalibratePoint(0, NULL, &faims.AUXHVmonI, &V2);
  float m = (float)(adcV2-adcV1) / (V2-V1);
  float b = (float)adcV1 - V1 * m;
  serial->println("ADC channel calibration parameters.");
  serial->print("m = ");
  serial->println(m);
  serial->print("b = ");
  serial->println(b);
  faims.AUXHVmonI.m = m;
  faims.AUXHVmonI.b = b;
  faims.Voltage = V;
  UpdateDACvalue(HVDACV, &faims.HVvoltCtrl, &faims.Voltage, &sdata.Voltage, true);  
}
