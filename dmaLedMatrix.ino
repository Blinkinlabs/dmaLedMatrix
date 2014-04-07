//Geometry
#define LED_COLS 32
#define LED_ROWS 16
#define ROWS_PER_OUTPUT 2
#define BIT_DEPTH 9

#define ANIMATION_COUNTS 1

// Output assignments
#define LED_R0   15    // Port C, output 0
#define LED_G0   22    // Port C, output 1
#define LED_B0   23    // Port C, output 2
#define LED_R1    9    // Port C, output 3
#define LED_G1   10    // Port C, output 4
#define LED_B1   13    // Port C, output 5
//#define LED_A     0
//#define LED_B     1
//#define LED_C     2
//#define LED_D     3
#define LED_A     16   // Port B, output 0
#define LED_B     17   // Port B, output 1
#define LED_C     19   // Port B, output 2
#define LED_D     18   // Port B, output 3
#define LED_CLK  11    // Port C, output 6
#define LED_STB  12    // Port C, output 7
#define LED_OE    4

// Offsets in the port b register
#define DMA_ADDRESS_A_SHIFT 0
#define DMA_ADDRESS_B_SHIFT 1
#define DMA_ADDRESS_C_SHIFT 3
#define DMA_ADDRESS_D_SHIFT 2

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

// Address output buffer
uint8_t Addresses[LED_ROWS/ROWS_PER_OUTPUT];

// Display buffer (standard RGB array!)
pixel Pixels[LED_COLS * LED_ROWS];


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
  
  // Enable DMA clock
  SIM_SCGC7 |= SIM_SCGC7_DMA;

  // Use default configuration
  DMA_CR = 0;
  
  makeFadeCircle(Pixels, 16, 8);
  pixelsToDmaBuffer(Pixels, DmaBuffer[0]);
  
  // Fill the address table
  for(int address = 0; address < LED_ROWS/ROWS_PER_OUTPUT; address++) {
    Addresses[address] = address;
  }
}

// TCD1 clocks and strobes the pixel data, which are on port C
void setupTCD1(uint8_t* source, int minorLoopSize, int majorLoops) {
  // DMA channel #1 responsable for shifting out the data/clock stream
  DMA_TCD1_SADDR = source;                                        // Address to read from
  DMA_TCD1_SOFF = 1;                                              // Bytes to increment source register between writes 
  DMA_TCD1_ATTR = DMA_TCD_ATTR_SSIZE(0) | DMA_TCD_ATTR_DSIZE(0);  // 8-bit input and output
  DMA_TCD1_NBYTES_MLNO = minorLoopSize;                           // Number of bytes to transfer in the minor loop
  DMA_TCD1_SLAST = 0;                                             // Bytes to add after a major iteration count (N/A)
  DMA_TCD1_DADDR = &GPIOC_PDOR;                                   // Address to write to
  DMA_TCD1_DOFF = 0;                                              // Bytes to increment destination register between write
  DMA_TCD1_CITER_ELINKNO = majorLoops;                            // Number of major loops to complete
  DMA_TCD1_BITER_ELINKNO = majorLoops;                            // Reset value for CITER (must be equal to CITER)
  DMA_TCD1_DLASTSGA = 0;                                          // Address of next TCD (N/A)
}

// TCD2 writes out the address select lines, which are on port B
void setupTCD2(uint8_t* source, int minorLoopSize, int majorLoops) {
  // DMA channel #1 responsable for shifting out the data/clock stream
  DMA_TCD2_SADDR = source;                                        // Address to read from
  DMA_TCD2_SOFF = 1;                                              // Bytes to increment source register between writes 
  DMA_TCD2_ATTR = DMA_TCD_ATTR_SSIZE(0) | DMA_TCD_ATTR_DSIZE(0);  // 8-bit input and output
  DMA_TCD2_NBYTES_MLNO = minorLoopSize;                           // Number of bytes to transfer in the minor loop
  DMA_TCD2_SLAST = 0;                                             // Bytes to add after a major iteration count (N/A)
  DMA_TCD2_DADDR = &GPIOB_PDOR;                                   // Address to write to
  DMA_TCD2_DOFF = 0;                                              // Bytes to increment destination register between write
  DMA_TCD2_CITER_ELINKNO = majorLoops;                            // Number of major loops to complete
  DMA_TCD2_BITER_ELINKNO = majorLoops;                            // Reset value for CITER (must be equal to CITER)
  DMA_TCD2_DLASTSGA = 0;                                          // Address of next TCD (N/A)
}


float animationX = 0;
float animationY = 0;
int counts = 0;
int frame = 0;

float xSpeed = .14;
float ySpeed = .18;

void loop() {

//  uint8_t* dataPointer = DmaBuffer[frame];
  setupTCD1(DmaBuffer[frame], ROW_BIT_SIZE, BIT_DEPTH*LED_ROWS/ROWS_PER_OUTPUT);


  setupTCD2(Addresses, 1, LED_ROWS/ROWS_PER_OUTPUT);

  // Now, start cycling through the rows
  for(int row = 0; row < (LED_ROWS/ROWS_PER_OUTPUT); row++) {

//    // Set up the address outputs (todo: Delay this?)
//    digitalWrite(LED_A, (row >> 0) & 0x01);
//    digitalWrite(LED_B, (row >> 1) & 0x01);
//    digitalWrite(LED_C, (row >> 2) & 0x01);
    
    DMA_TCD2_CSR |= DMA_TCD_CSR_START;                              // Start the transfer
    while ((DMA_TCD2_CSR & DMA_TCD_CSR_ACTIVE));                     // Wait for the minor loop to complete
  
    // For this row, write out all the bit depths
    for(int depth = 0; depth < BIT_DEPTH; depth++) {
      
//      // Write out this bit depth worth of data
//      for(int step = 0; step < ROW_BIT_SIZE; step++) {
//        GPIOC_PDOR = *dataPointer;
//        dataPointer++;
//      }

      DMA_TCD1_CSR |= DMA_TCD_CSR_START;                              // Start the transfer
      while ((DMA_TCD1_CSR & DMA_TCD_CSR_ACTIVE));                     // Wait for the minor loop to complete
  
      digitalWrite(LED_OE, LOW);
      delayMicroseconds(1 * (1 << depth));
      digitalWrite(LED_OE, HIGH);
    }
  }
  
  counts++;
  if(counts > ANIMATION_COUNTS) {
    counts = 0;
    animationX = (animationX + xSpeed);
    if(animationX > LED_COLS*2) {
      animationX -= LED_COLS*2;
    }

    animationY = (animationY + ySpeed);
    if(animationY > LED_ROWS*2) {
      animationY -= LED_ROWS*2;
    }
    
    makeFadeCircle(Pixels, abs(LED_COLS - animationX), abs(LED_ROWS - animationY));
    pixelsToDmaBuffer(Pixels, DmaBuffer[frame]);
  }
}
