// Use SAMD51's DMAC to generate a 1kHz sine wave 0 to 3.3V amplitude on A0 using DAC0
#define SAMPLE_NO 1000
#define BTN_A 9
#define BTN_B 6
#define BTN_C 5
#define NUM_FREQS 10
#define NUM_AMPS 10
#define CYCLES(f) ((48000000/(f*1000))-1)
#define DEBOUNCE_DELAY 200 //ms

uint16_t sintable[SAMPLE_NO];                                             // Sine table
uint16_t sintable2[SAMPLE_NO];                                             // Sine table

typedef struct {
  uint16_t numOptions;
  float* options;
  uint16_t idx;
  unsigned long lastChange;
} attribute;

attribute* setupAttribute(uint16_t numOptions, float minOption, float maxOption) { 
  attribute* attr = (attribute*) malloc(sizeof(attribute));
  if(attr) {
    attr->numOptions = numOptions;
    attr->options = new float[attr->numOptions];
    for (int i=0; i<attr->numOptions; i++) {
      attr->options[i] = minOption + (((maxOption-minOption)/(attr->numOptions-1)) * i);
    }
    attr->idx = 0;
    attr->lastChange = millis();
  } else {
    Serial.println("Could not malloc attribute");
  }
  return attr;
}

void destroyAttribute(attribute* attr) {
    if(attr) free(attr);
}

void printAttribute(attribute *attr) {
  Serial.println("numOptions: " + (String) attr->numOptions);
  Serial.print("Options[] = {");
  for (int i=0; i<attr->numOptions; i++) {
    Serial.print((String) attr->options[i]);
    if ((i+1) < attr->numOptions) {
      Serial.print(", ");
    }
  }
  Serial.print("}\n");
  Serial.println("idx: " + (String) attr->idx);
  Serial.println("lastChange: " + (String) attr->lastChange);
}

bool actuallyPressed(attribute *attr) {
  unsigned long now = millis();
  if (now - attr->lastChange >= DEBOUNCE_DELAY) {
    attr->lastChange = now;
    return true;
  }  
  return false;
}

void nextOption(attribute *attr) {
  if (++(attr->idx) >= attr->numOptions)
    attr->idx = 0;
}

float getValue(attribute *attr) {
  return attr->options[attr->idx];
}

attribute *freq, *amp, *phase;

void updateFrequency() {
  TC0->COUNT16.CC[0].reg = CYCLES(getValue(freq));         // Set the sine wave frequency to 1kHz: 48MHz / (1000 * 1000) - 1
  while (TC0->COUNT16.SYNCBUSY.bit.CC0);                     // Wait for synchronization
}

void buildSinTable() {
  float a = getValue(amp);
  float p = getValue(phase);
  for (uint16_t i = 0; i < SAMPLE_NO; i++)                                // Calculate the sine table with 1000 entries
  {
    sintable[i] = (uint16_t)((sinf((2 * PI * (float)i / SAMPLE_NO) + p) * (2047.0f * a)) + (2048.0f * a));    // 12-bit resolution with +1.63V offset
    sintable2[i] = (uint16_t)((sinf((2 * PI * (float)i / SAMPLE_NO)) * (2047.0f * a)) + (2048.0f * a));    // 12-bit resolution with +1.63V offset

  }
}

typedef struct                                                            // DMAC descriptor structure
{
  uint16_t btctrl;
  uint16_t btcnt;
  uint32_t srcaddr;
  uint32_t dstaddr;
  uint32_t descaddr;
} dmacdescriptor ;

volatile dmacdescriptor wrb[DMAC_CH_NUM] __attribute__ ((aligned (16)));          // Write-back DMAC descriptors
dmacdescriptor descriptor_section[DMAC_CH_NUM] __attribute__ ((aligned (16)));    // DMAC channel descriptors
dmacdescriptor descriptor __attribute__ ((aligned (16))); 

void aISR() {
  // Serial.println("aISR()");
  if (! actuallyPressed(freq)) return;
  nextOption(freq);
  updateFrequency();
  Serial.printf("Freq: %f\n", getValue(freq));
}
void bISR(){
  // Serial.println("bISR()");
  if (! actuallyPressed(amp)) return;
  nextOption(amp);
  buildSinTable();
  Serial.printf("Amp: %f\n", getValue(amp));
} 
void cISR(){
  // Serial.println("cISR()");
  if (! actuallyPressed(phase)) return;
  nextOption(phase);
  buildSinTable();
  Serial.printf("Phase: %f\n", getValue(phase));
}

void setup() {
  Serial.begin(9600);
  while (!Serial);
  delay(50);
  Serial.println("\nserial set up");

  freq = setupAttribute(10, 300, 1200);
  printAttribute(freq);
  amp = setupAttribute(10, 0.1, 1);
  printAttribute(amp);
  phase = setupAttribute(10, 0, 2*PI);
  printAttribute(phase);

  Serial.println("Built attributes");

  DMAC->BASEADDR.reg = (uint32_t)descriptor_section;                      // Specify the location of the descriptors
  DMAC->WRBADDR.reg = (uint32_t)wrb;                                      // Specify the location of the write back descriptors
  DMAC->CTRL.reg = DMAC_CTRL_DMAENABLE | DMAC_CTRL_LVLEN(0xf);            // Enable the DMAC peripheral

  buildSinTable();
  
  DAC->DACCTRL[0].bit.CCTRL = 1;                                          // Set the DAC's current control to allow output to operate at 1MSPS
  DAC->DACCTRL[1].bit.CCTRL = 1;                                          // Set the DAC's current control to allow output to operate at 1MSPS
  analogWriteResolution(12);                                              // Set the DAC's resolution to 12-bits
  analogWrite(A0, 0);                                                     // Initialise DAC0
  analogWrite(A1, 0);                                                     // Initialise DAC0

 
  DMAC->Channel[5].CHCTRLA.reg = DMAC_CHCTRLA_TRIGSRC(TC0_DMAC_ID_OVF) |  // Set DMAC to trigger when TC0 timer overflows
                                 DMAC_CHCTRLA_TRIGACT_BURST;              // DMAC burst transfer
  descriptor.descaddr = (uint32_t)&descriptor_section[5];                 // Set up a circular descriptor
  descriptor.srcaddr = (uint32_t)&sintable[0] + SAMPLE_NO * sizeof(uint16_t);  // Read the current value in the sine table
  descriptor.dstaddr = (uint32_t)&DAC->DATA[0].reg;                       // Copy it into the DAC data register
  descriptor.btcnt = SAMPLE_NO;                                                // This takes the number of sine table entries = 1000 beats
  descriptor.btctrl = DMAC_BTCTRL_BEATSIZE_HWORD |                        // Set the beat size to 16-bits (Half Word)
                      DMAC_BTCTRL_SRCINC |                                // Increment the source address every beat
                      DMAC_BTCTRL_VALID;                                  // Flag the descriptor as valid
  memcpy((void*)&descriptor_section[5], &descriptor, sizeof(dmacdescriptor));  // Copy to the channel 5 descriptor 

/*
 *  2nd DAC
 */
  DMAC->Channel[6].CHCTRLA.reg = DMAC_CHCTRLA_TRIGSRC(TC0_DMAC_ID_OVF) |  // Set DMAC to trigger when TC0 timer overflows
                                 DMAC_CHCTRLA_TRIGACT_BURST;              // DMAC burst transfer
  descriptor.descaddr = (uint32_t)&descriptor_section[6];                 // Set up a circular descriptor
  descriptor.srcaddr = (uint32_t)&sintable2[0] + SAMPLE_NO * sizeof(uint16_t);  // Read the current value in the sine table
  descriptor.dstaddr = (uint32_t)&DAC->DATA[1].reg;                       // Copy it into the DAC data register
  memcpy((void*)&descriptor_section[6], &descriptor, sizeof(dmacdescriptor));  // Copy to the channel 5 descriptor

  GCLK->PCHCTRL[TC0_GCLK_ID].reg = GCLK_PCHCTRL_CHEN |                     // Enable perhipheral channel for TC0
                                   GCLK_PCHCTRL_GEN_GCLK1;                // Connect generic clock 1 at 48MHz
 //

  TC0->COUNT16.WAVE.reg = TC_WAVE_WAVEGEN_MFRQ;                           // Set TC0 to Match Frequency (MFRQ) mode

  updateFrequency();                               

  TC0->COUNT16.CTRLA.bit.ENABLE = 1;                                      // Enable the TC0 timer
  while (TC0->COUNT16.SYNCBUSY.bit.ENABLE);                               // Wait for synchronization
 
  DMAC->Channel[5].CHCTRLA.bit.ENABLE = 1;                                // Enable DMAC on channel 5
  DMAC->Channel[6].CHCTRLA.bit.ENABLE = 1;                                // Enable DMAC on channel 5


  pinMode(BTN_A, INPUT_PULLUP);
  pinMode(BTN_B, INPUT_PULLUP);
  pinMode(BTN_C, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BTN_A), aISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(BTN_B), bISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(BTN_C), cISR, FALLING);
  Serial.println("Button pins set up");
}

void loop(){}