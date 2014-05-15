#include <cmath>

//Display Geometry
#define LED_COLS 32
#define LED_ROWS 16
#define ROWS_PER_OUTPUT 2
#define BIT_DEPTH 8

// Output assignments
#define LED_R0   15    // Port C, output 0
#define LED_G0   22    // Port C, output 1
#define LED_B0   23    // Port C, output 2
#define LED_R1    9    // Port C, output 3
#define LED_G1   10    // Port C, output 4
#define LED_B1   13    // Port C, output 5
#define LED_A     16   // Port B, output 0
#define LED_B     17   // Port B, output 1
#define LED_C     19   // Port B, output 2
#define LED_D     18   // Port B, output 3
#define LED_CLK  11    // Port C, output 6
#define LED_STB  12    // Port C, output 7
#define LED_OE    3    // FTM1 channel 0

#define MAJOR_INT_FLAG 2 // TODO: Delete me

// Offsets in the port c register
#define DMA_R0_SHIFT   0
#define DMA_G0_SHIFT   1
#define DMA_B0_SHIFT   2
#define DMA_R1_SHIFT   3
#define DMA_G1_SHIFT   4
#define DMA_B1_SHIFT   5
#define DMA_CLK_SHIFT  6
#define DMA_STB_SHIFT  7

struct pixel {
  uint16_t R;
  uint16_t G;
  uint16_t B;
};

struct quadrantColorMultiplier {
  uint8_t R;
  uint8_t G;
  uint8_t B;
  float brightness;
};

// Display buffer (write into this!)
pixel Pixels[LED_COLS * LED_ROWS];

// Address output buffer
uint8_t Addresses[BIT_DEPTH*LED_ROWS/ROWS_PER_OUTPUT];

// Timer output buffers (these will be DMAd to the FTM1_MOD and FTM1_C0V registers)
uint32_t FTM1_MODStates[BIT_DEPTH*LED_ROWS/ROWS_PER_OUTPUT];
uint32_t FTM1_C0VStates[BIT_DEPTH*LED_ROWS/ROWS_PER_OUTPUT];

// Big 'ol waveform that should be sent out over DMA in chunks.
// There are LED_ROWS/ROWS_PER_OUTPUT separate loops, where the LED matrix address lines
// to be set before they are activated.
// For each of these rows, there are then BIT_DEPTH separate inner loops
// And each inner loop has LED_COLS * 2 bytes states (the data is LED_COLS long, plus the clock signal is baked in)

#define ROW_BIT_SIZE (LED_COLS*2 + 2)                              // Number of bytes required to store a single row of 1-bit color data output
#define ROW_DEPTH_SIZE (ROW_BIT_SIZE*BIT_DEPTH)                    // Number of bytes required to store a single row of full-color data output
#define PANEL_DEPTH_SIZE (ROW_DEPTH_SIZE*LED_ROWS/ROWS_PER_OUTPUT) // Number of bytes required to store an entire panel's worth of data output.

// 2x DMA buffer
uint8_t DmaBuffer[2][PANEL_DEPTH_SIZE];

// Munge the data so it can be written out by the DMA engine
// Note: bufferOutput[][xxx] should have BIT_DEPTH as xxx
void pixelsToDmaBuffer(struct pixel* pixelInput, uint8_t bufferOutput[]) {

  // Fill in the pixel data
  for(int row = 0; row < LED_ROWS/ROWS_PER_OUTPUT; row++) {
    for(int col = 0; col < LED_COLS; col++) {
      int data_R0 = pixelInput[row*LED_COLS                              + col].R;
      int data_G0 = pixelInput[row*LED_COLS                              + col].G;
      int data_B0 = pixelInput[row*LED_COLS                              + col].B;
      int data_R1 = pixelInput[(row + LED_ROWS/ROWS_PER_OUTPUT)*LED_COLS + col].R;
      int data_G1 = pixelInput[(row + LED_ROWS/ROWS_PER_OUTPUT)*LED_COLS + col].G;
      int data_B1 = pixelInput[(row + LED_ROWS/ROWS_PER_OUTPUT)*LED_COLS + col].B;

      for(int depth = 0; depth < BIT_DEPTH; depth++) {
        uint8_t output = (((data_R0 >> depth) & 0x01) << DMA_R0_SHIFT)
          | (((data_G0 >> depth) & 0x01) << DMA_G0_SHIFT)
            | (((data_B0 >> depth) & 0x01) << DMA_B0_SHIFT)
              | (((data_R1 >> depth) & 0x01) << DMA_R1_SHIFT)
                | (((data_G1 >> depth) & 0x01) << DMA_G1_SHIFT)
                  | (((data_B1 >> depth) & 0x01) << DMA_B1_SHIFT);

        bufferOutput[row*ROW_DEPTH_SIZE + depth*ROW_BIT_SIZE + col*2 + 0] = output;
        bufferOutput[row*ROW_DEPTH_SIZE + depth*ROW_BIT_SIZE + col*2 + 1] = output | 1 << DMA_CLK_SHIFT;
      }
    }
  }

  // Fill in the strobe data
  for(int row = 0; row < LED_ROWS/ROWS_PER_OUTPUT; row++) {
    for(int depth = 0; depth < BIT_DEPTH; depth++) {
      bufferOutput[row*ROW_DEPTH_SIZE + depth*ROW_BIT_SIZE + LED_COLS*2 + 0] = 1 << DMA_STB_SHIFT;
      bufferOutput[row*ROW_DEPTH_SIZE + depth*ROW_BIT_SIZE + LED_COLS*2 + 1] = 0 << DMA_STB_SHIFT;
    } 
  }
}



void makeRed(struct pixel* pixelInput, float x, float y) {
  int rVal;
  int gVal;
  int bVal;

  // Draw a frame and set to go.
  for(int row = 0; row < LED_ROWS; row++) {
    for(int col = 0; col < LED_COLS; col++) {
      pixelInput[row*LED_COLS + col].R = 0xFFFF; //(1 << (BIT_DEPTH-1));
      pixelInput[row*LED_COLS + col].G = 0xFFFF;
      pixelInput[row*LED_COLS + col].B = 0xFFFF;

      //      pixelInput[row*LED_COLS + col].R = ((col&1)?0x5555:0);
      //      pixelInput[row*LED_COLS + col].G = ((col&1)?0x5555:0);
      //      pixelInput[row*LED_COLS + col].B = ((col&1)?0x5555:0);
    }
  }
}

void makeFadeCircle(struct pixel* pixelInput, float x, float y) {
  int rVal;
  int gVal;
  int bVal;

  // Draw a frame and set to go.
  for(int row = 0; row < LED_ROWS; row++) {
    for(int col = 0; col < LED_COLS; col++) {
      int val = min(abs(row - y) * abs(col - x) * 1, (1 << BIT_DEPTH) - 1);

      if(row > y && col > x) {
        rVal = val;
        gVal = val;
        bVal = 0;
      }
      else if(row > y && col <x) {
        rVal = 0;
        gVal = val;
        bVal = val;
      }
      else if(row < y && col >x) {
        rVal = val;
        gVal = 0;
        bVal = val;
      }
      else {
        rVal = val;
        gVal = val;
        bVal = val;
      }

      pixelInput[row*LED_COLS + col].R = rVal;
      pixelInput[row*LED_COLS + col].G = gVal;
      pixelInput[row*LED_COLS + col].B = bVal;
    }
  }
}

int quadSize = 12;

int quadCount = 9; // Note: fixed in the h. file, lazy here.
int quadCountX = 3;
int quadCountY = 3;

quadrantColorMultiplier QuadColors[9];

void makeScrollingBoxes(struct pixel* pixelInput, float x, float y) {
  // Draw a frame and set to go.
  for(int row = 0; row < LED_ROWS; row++) {
    for(int col = 0; col < LED_COLS; col++) {

      //int val = std::min(abs(row - y) * abs(col - x) * 1, (1 << BIT_DEPTH) - 1);
      //int val = abs(row - y) * abs(col - x) * 1;
      //int val = 255;
      //std::cout << val << std::endl;

      while(row - x < 0) { 
        x -= quadSize*quadCount;
      }
      while(col - y < 0) { 
        y -= quadSize*quadCount;
      }

      int xQuad = ((int(col - x)) / quadSize) % quadCountX;
      int yQuad = ((int(row - y)) / quadSize) % quadCountY;
      int quad = yQuad*quadCountX + xQuad;

      float quadPosX = quadSize/2 - std::fabs(std::fmod(col-x,quadSize)-(quadSize/2));
      float quadPosY = quadSize/2 - std::fabs(std::fmod(row-y,quadSize)-(quadSize/2));

      float val = quadPosX*quadPosY*8;

      pixelInput[row*LED_COLS + col].R = min(int(val*QuadColors[quad].R/255.0*std::fabs(QuadColors[quad].brightness-1)),255);
      pixelInput[row*LED_COLS + col].G = min(int(val*QuadColors[quad].G/255.0*std::fabs(QuadColors[quad].brightness-1)),255);
      pixelInput[row*LED_COLS + col].B = min(int(val*QuadColors[quad].B/255.0*std::fabs(QuadColors[quad].brightness-1)),255);
    }
  }

  for(int i = 0; i < quadCount; i++) {
    QuadColors[i].brightness = std::fmod(QuadColors[i].brightness + .02, 2);
  }
}

void makeSparkles(struct pixel* pixelInput, float x, float y) {
  for(int row = 0; row < LED_ROWS; row++) {
    for(int col = 0; col < LED_COLS; col++) {
      if(random(2)) {
        pixelInput[row*LED_COLS + col].R = 65535;
        pixelInput[row*LED_COLS + col].G = 65535;
        pixelInput[row*LED_COLS + col].B = 65535;
      }
      else {
        pixelInput[row*LED_COLS + col].R = 0;
        pixelInput[row*LED_COLS + col].G = 0;
        pixelInput[row*LED_COLS + col].B = 0;
      }        
    }
  }
}

// TCD0 updates the timer values for FTM1
void setupTCD0(uint32_t* source, int minorLoopSize, int majorLoops) {
  DMA_TCD0_SADDR = source;                                        // Address to read from
  DMA_TCD0_SOFF = 4;                                              // Bytes to increment source register between writes 
  DMA_TCD0_ATTR = DMA_TCD_ATTR_SSIZE(2) | DMA_TCD_ATTR_DSIZE(2);  // 32-bit input and output
  DMA_TCD0_NBYTES_MLNO = minorLoopSize;                           // Number of bytes to transfer in the minor loop
  DMA_TCD0_SLAST = 0;                                             // Bytes to add after a major iteration count (N/A)
  //  DMA_TCD0_DADDR = TimerStatesDump;                               // Address to write to
  DMA_TCD0_DADDR = &FTM1_MOD;                                      // Address to write to
  DMA_TCD0_DOFF = 0;                                              // Bytes to increment destination register between write
  //  DMA_TCD0_CITER_ELINKNO = majorLoops;                            // Number of major loops to complete
  //  DMA_TCD0_BITER_ELINKNO = majorLoops;                            // Reset value for CITER (must be equal to CITER)
  DMA_TCD0_DLASTSGA = 0;                                          // Address of next TCD (N/A)

  // Workaround for DMA majorelink unreliability: increase the minor loop count by one
  // Note that the final transfer doesn't end up happening, because 
  DMA_TCD0_CITER_ELINKYES = majorLoops + 1;                           // Number of major loops to complete
  DMA_TCD0_BITER_ELINKYES = majorLoops + 1;                           // Reset value for CITER (must be equal to CITER)

  // Trigger DMA1 (timer) after each minor loop
  DMA_TCD0_BITER_ELINKYES |= DMA_TCD_CITER_ELINK;
  DMA_TCD0_BITER_ELINKYES |= (0x01 << 9);  
  DMA_TCD0_CITER_ELINKYES |= DMA_TCD_CITER_ELINK;
  DMA_TCD0_CITER_ELINKYES |= (0x01 << 9);
}

// TCD1 updates the timer values for FTM1
void setupTCD1(uint32_t* source, int minorLoopSize, int majorLoops) {
  DMA_TCD1_SADDR = source;                                        // Address to read from
  DMA_TCD1_SOFF = 4;                                              // Bytes to increment source register between writes 
  DMA_TCD1_ATTR = DMA_TCD_ATTR_SSIZE(2) | DMA_TCD_ATTR_DSIZE(2);  // 32-bit input and output
  DMA_TCD1_NBYTES_MLNO = minorLoopSize;                           // Number of bytes to transfer in the minor loop
  DMA_TCD1_SLAST = 0;                                             // Bytes to add after a major iteration count (N/A)
  //  DMA_TCD0_DADDR = TimerStatesDump;                               // Address to write to
  DMA_TCD1_DADDR = &FTM1_C0V;                                      // Address to write to
  DMA_TCD1_DOFF = 0;                                              // Bytes to increment destination register between write
  //  DMA_TCD1_CITER_ELINKNO = majorLoops;                            // Number of major loops to complete
  //  DMA_TCD1_BITER_ELINKNO = majorLoops;                            // Reset value for CITER (must be equal to CITER)
  DMA_TCD1_DLASTSGA = 0;                                          // Address of next TCD (N/A)

  // Workaround for DMA majorelink unreliability: increase the minor loop count by one
  // Note that the final transfer doesn't end up happening, because 
  DMA_TCD1_CITER_ELINKYES = majorLoops + 1;                           // Number of major loops to complete
  DMA_TCD1_BITER_ELINKYES = majorLoops + 1;                           // Reset value for CITER (must be equal to CITER)

  // Trigger DMA2 (data) after each minor loop
  DMA_TCD1_BITER_ELINKYES |= DMA_TCD_CITER_ELINK;
  DMA_TCD1_BITER_ELINKYES |= (0x02 << 9);  
  DMA_TCD1_CITER_ELINKYES |= DMA_TCD_CITER_ELINK;
  DMA_TCD1_CITER_ELINKYES |= (0x02 << 9);
}

// TCD2 clocks and strobes the pixel data, which are on port C
void setupTCD2(uint8_t* source, int minorLoopSize, int majorLoops) {
  DMA_TCD2_SADDR = source;                                        // Address to read from
  DMA_TCD2_SOFF = 1;                                              // Bytes to increment source register between writes 
  DMA_TCD2_ATTR = DMA_TCD_ATTR_SSIZE(0) | DMA_TCD_ATTR_DSIZE(0);  // 8-bit input and output
  DMA_TCD2_NBYTES_MLNO = minorLoopSize;                           // Number of bytes to transfer in the minor loop
  DMA_TCD2_SLAST = 0;                                             // Bytes to add after a major iteration count (N/A)
  DMA_TCD2_DADDR = &GPIOC_PDOR;                                   // Address to write to
  DMA_TCD2_DOFF = 0;                                              // Bytes to increment destination register between write
  //  DMA_TCD2_CITER_ELINKNO = majorLoops;                            // Number of major loops to complete
  //  DMA_TCD2_BITER_ELINKNO = majorLoops;                            // Reset value for CITER (must be equal to CITER)
  DMA_TCD2_DLASTSGA = 0;                                          // Address of next TCD (N/A)

  // Workaround for DMA majorelink unreliability: increase the minor loop count by one
  // Note that the final transfer doesn't end up happening, because 
  DMA_TCD2_CITER_ELINKYES = majorLoops + 1;                           // Number of major loops to complete
  DMA_TCD2_BITER_ELINKYES = majorLoops + 1;                           // Reset value for CITER (must be equal to CITER)

  // Trigger DMA3 (address) after each minor loop
  DMA_TCD2_BITER_ELINKYES |= DMA_TCD_CITER_ELINK;
  DMA_TCD2_BITER_ELINKYES |= (0x03 << 9);  
  DMA_TCD2_CITER_ELINKYES |= DMA_TCD_CITER_ELINK;
  DMA_TCD2_CITER_ELINKYES |= (0x03 << 9);
}


// TCD3 writes out the address select lines, which are on port B
void setupTCD3(uint8_t* source, int minorLoopSize, int majorLoops) {
  DMA_TCD3_SADDR = source;                                        // Address to read from
  DMA_TCD3_SOFF = 1;                                              // Bytes to increment source register between writes 
  DMA_TCD3_ATTR = DMA_TCD_ATTR_SSIZE(0) | DMA_TCD_ATTR_DSIZE(0);  // 8-bit input and output
  DMA_TCD3_NBYTES_MLNO = minorLoopSize;                           // Number of bytes to transfer in the minor loop
  DMA_TCD3_SLAST = 0;                                             // Bytes to add after a major iteration count (N/A)
  DMA_TCD3_DADDR = &GPIOB_PDOR;                                   // Address to write to
  DMA_TCD3_DOFF = 0;                                              // Bytes to increment destination register between write
  DMA_TCD3_CITER_ELINKYES = majorLoops;                           // Number of major loops to complete
  DMA_TCD3_BITER_ELINKYES = majorLoops;                           // Reset value for CITER (must be equal to CITER)
  DMA_TCD3_DLASTSGA = 0;                                          // Address of next TCD (N/A)
}

// When the last address write has completed, that means we're at the end of the display refresh cycle
// Set up the next display frame
// TODO: flip pages, etc
void dma_ch3_isr(void) {
  DMA_CINT = 3;
  digitalWrite(MAJOR_INT_FLAG, HIGH);  // TODO: Delete me
  setupTCDs();

  digitalWrite(MAJOR_INT_FLAG, LOW);  // TODO: Delete me
}

void setupTCDs() {
  //  setupTCD0(TimerStates,  8*4,          BIT_DEPTH*LED_ROWS/ROWS_PER_OUTPUT);
  setupTCD0(FTM1_MODStates,  4,         BIT_DEPTH*LED_ROWS/ROWS_PER_OUTPUT);
  setupTCD1(FTM1_C0VStates,  4,         BIT_DEPTH*LED_ROWS/ROWS_PER_OUTPUT);
  setupTCD2(DmaBuffer[0], ROW_BIT_SIZE, BIT_DEPTH*LED_ROWS/ROWS_PER_OUTPUT);
  setupTCD3(Addresses,    1,            BIT_DEPTH*LED_ROWS/ROWS_PER_OUTPUT);
}


// FTM1 drives our whole operation! We need to periodically update the
// FTM1_MOD and FTM1_C1V registers to program the next cycle.
void setupFTM1(){
  FTM1_MODE = FTM_MODE_WPDIS;    // Disable Write Protect

  FTM1_SC = 0;                   // Turn off the clock so we can update CNTIN and MODULO?
  FTM1_MOD = 0x02FF;             // Period register
  FTM1_SC |= FTM_SC_CLKS(1) | FTM_SC_PS(1);

  FTM1_MODE |= FTM_MODE_INIT;         // Enable FTM0

  FTM1_C0SC = 0x40                    // Enable interrupt
  | 0x20                    // Mode select: Edge-aligned PWM 
  | 0x04                    // Low-true pulses (inverted)
  | 0x01;                   // Enable DMA out
  FTM1_C0V = 0x0200;                       // Duty cycle of PWM signal
  FTM1_SYNC |= 0x80;                  // set PWM value update


  // Configure LED_OE pinmux (LED_OE is on FTM1_CH1)`
  CORE_PIN3_CONFIG = PORT_PCR_MUX(3) | PORT_PCR_DSE | PORT_PCR_SRE; 
}

void setup() {
  pinMode(LED_R0, OUTPUT);
  pinMode(LED_G0, OUTPUT);
  pinMode(LED_B0, OUTPUT);
  pinMode(LED_R1, OUTPUT);
  pinMode(LED_G1, OUTPUT);
  pinMode(LED_B1, OUTPUT);

  pinMode(LED_A, OUTPUT);
  pinMode(LED_B, OUTPUT);
  pinMode(LED_C, OUTPUT);
  pinMode(LED_D, OUTPUT);
  pinMode(LED_CLK, OUTPUT);
  pinMode(LED_STB, OUTPUT);
  pinMode(LED_OE, OUTPUT);

  pinMode(MAJOR_INT_FLAG, OUTPUT); // TODO: Delete me

  // Fill the address table
  // To make the DMA engine easier to program, we store a copy of the address table for each output page.
  for(int address = 0; address < LED_ROWS/ROWS_PER_OUTPUT; address++) {
    for(int page = 0; page < BIT_DEPTH; page++) {
      Addresses[address*BIT_DEPTH + page] = address;
    }
  }

  // Set D high on the last address, for use in debugging (TODO: Delete me!)
  Addresses[BIT_DEPTH*LED_ROWS/ROWS_PER_OUTPUT-1] |= 0x08;


  // Fill the timer states table
  for(int address = 0; address < LED_ROWS/ROWS_PER_OUTPUT; address++) {
    int onTime = 0x004F;
    int blankingTime = 0xA4;  // Time that OE is disabled, when the timer update, data, and address updates are made

    for(int page = 0; page < BIT_DEPTH; page++) {      

      FTM1_C0VStates[address*BIT_DEPTH + page] = onTime;      
      FTM1_MODStates[address*BIT_DEPTH + page] = onTime + blankingTime;

      onTime = onTime*2;

      //      TimerStates[(address*BIT_DEPTH + page)*TIMER_STATES_SIZE + 0] =                  // FTM1_SC
      //              FTM_SC_CLKS(1) | FTM_SC_PS(1);  //Clock setup
      //      TimerStates[(address*BIT_DEPTH + page)*TIMER_STATES_SIZE + 1] = 0;               // FTM1_CNT
      //      TimerStates[(address*BIT_DEPTH + page)*TIMER_STATES_SIZE + 2] = onTime + 0x58;   // FTM1_MOD
      //      TimerStates[(address*BIT_DEPTH + page)*TIMER_STATES_SIZE + 3] =                  // FTM1_C0SC
      //              0x40                    // Enable interrupt
      //            | 0x20                    // Mode select: Edge-aligned PWM 
      //            | 0x04                    // Low-true pulses (inverted)
      //            | 0x01;                   // Enable DMA out
      //      TimerStates[(address*BIT_DEPTH + page)*TIMER_STATES_SIZE + 4] = onTime;          // FTM1_C0V
      //      TimerStates[(address*BIT_DEPTH + page)*TIMER_STATES_SIZE + 5] = 0;               // FTM1_C1SC
      //      TimerStates[(address*BIT_DEPTH + page)*TIMER_STATES_SIZE + 6] = 0;               // FTM1_C1V      
      //      TimerStates[(address*BIT_DEPTH + page)*TIMER_STATES_SIZE + 7] = 0;               // N/A??
    }
  }

  // DMA
  // Configure DMA
  SIM_SCGC7 |= SIM_SCGC7_DMA;  // Enable DMA clock
  DMA_CR = 0;  // Use default configuration

  // Enable interrupt on major completion for DMA channel 2 (address)
  // (TODO: String data DMA after this, and skip the interrupt?)
  DMA_TCD3_CSR = DMA_TCD_CSR_INTMAJOR;                           // Enable interrupt on major complete
  NVIC_ENABLE_IRQ(IRQ_DMA_CH3);  // Enable interrupt request

  // Configure the DMA request input for DMA0, DMA1, DMA2
  // TODO: USE DMA_SERQ instead?

  DMA_ERQ |= DMA_ERQ_ERQ0; 
  //  DMA_ERQ |= DMA_ERQ_ERQ1;
  //  DMA_ERQ |= DMA_ERQ_ERQ2;

  // DMAMUX
  // Configure the DMAMUX
  SIM_SCGC6 |= SIM_SCGC6_DMAMUX; // Enable DMAMUX clock

  // Timer DMA channel:
  // Configure DMAMUX to trigger DMA0 from FTM1_CH0
  DMAMUX0_CHCFG0 = DMAMUX_DISABLE;
  DMAMUX0_CHCFG0 = DMAMUX_SOURCE_FTM1_CH0 | DMAMUX_ENABLE;

  // Data DMA channel:
  // Configure DMAMUX to trigger DMA1 from FTM1_CH0
  //  DMAMUX0_CHCFG1 = DMAMUX_DISABLE;
  //  DMAMUX0_CHCFG1 = DMAMUX_SOURCE_FTM1_CH0 | DMAMUX_ENABLE;

  // Address DMA channel:
  // Configure DMAMUX to trigger DMA2 from FTM1_CH0
  //  DMAMUX0_CHCFG2 = DMAMUX_DISABLE;
  //  DMAMUX0_CHCFG2 = DMAMUX_SOURCE_FTM1_CH0 | DMAMUX_ENABLE;

  // Kick off our animation
  makeFadeCircle(Pixels, 16, 8);
  pixelsToDmaBuffer(Pixels, DmaBuffer[0]);

  // Load this frame of data into the DMA engine
  setupTCDs();

  // FTM
  SIM_SCGC6 |= SIM_SCGC6_FTM1;  // Enable FTM0 clock
  setupFTM1();


  //Fill in the quad colors table
  QuadColors[0].R = 32;
  QuadColors[0].G = 64;
  QuadColors[0].B = 128;
  QuadColors[0].brightness = .6;

  QuadColors[1].R = 32;
  QuadColors[1].G = 128;
  QuadColors[1].B = 64;
  QuadColors[1].brightness = .4;

  QuadColors[2].R = 64;
  QuadColors[2].G = 32;
  QuadColors[2].B = 128;
  QuadColors[2].brightness = .7;

  QuadColors[3].R = 64;
  QuadColors[3].G = 128;
  QuadColors[3].B = 32;
  QuadColors[3].brightness = .9;

  QuadColors[4].R = 128;
  QuadColors[4].G = 32;
  QuadColors[4].B = 64;
  QuadColors[4].brightness = .3;

  QuadColors[5].R = 128;
  QuadColors[5].G = 64;
  QuadColors[5].B = 32;
  QuadColors[5].brightness = .5;

  QuadColors[6].R = 160;
  QuadColors[6].G = 32;
  QuadColors[6].B = 32;
  QuadColors[6].brightness = .1;

  QuadColors[7].R = 32;
  QuadColors[7].G = 160;
  QuadColors[7].B = 32;
  QuadColors[7].brightness = .8;

  QuadColors[8].R = 32;
  QuadColors[8].G = 32;
  QuadColors[8].B = 160;
  QuadColors[8].brightness = .2;
}


float animationX = 0;
float animationY = 0;
int counts = 0;
int frame = 0;

float xSpeed = .21;
float ySpeed = .24;

void loop() {

  counts++;
  if(counts > 2) {
    counts = 0;
    animationX = (animationX + xSpeed);
    //    if(animationX > LED_COLS*2) {
    //      animationX -= LED_COLS*2;
    //    }

    animationY = (animationY + ySpeed);
    //    if(animationY > LED_ROWS*2) {
    //      animationY -= LED_ROWS*2;
    //    }

    //makeRed(Pixels, abs(LED_COLS - animationX), abs(LED_ROWS - animationY));
    makeScrollingBoxes(Pixels, abs(LED_COLS - animationX), abs(LED_ROWS - animationY));
    //makeFadeCircle(Pixels, abs(LED_COLS - animationX), abs(LED_ROWS - animationY));
    
    pixelsToDmaBuffer(Pixels, DmaBuffer[frame]);
  }
}

