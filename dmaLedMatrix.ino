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
#define LED_A     0
#define LED_B     1
#define LED_C     2
#define LED_D     3
#define LED_CLK  11    // Port C, output 6
#define LED_STB   4
#define LED_OE    5

// Offsets in the port c register
#define DMA_R0_SHIFT   0
#define DMA_G0_SHIFT   1
#define DMA_B0_SHIFT   2
#define DMA_R1_SHIFT   3
#define DMA_G1_SHIFT   4
#define DMA_B1_SHIFT   5
#define DMA_CLK_SHIFT  6

struct pixel {
  uint16_t R;
  uint16_t G;
  uint16_t B;
};

// Size of the unrolled bitstream that describes each page
#define LED_BITPAGE_SIZE LED_COLS*LED_ROWS/ROWS_PER_OUTPUT*2

// Display buffer
pixel Pixels[LED_COLS * LED_ROWS];

// 2x DMA buffer
uint8_t DmaBuffer[2][BIT_DEPTH][LED_BITPAGE_SIZE];

// Munge the data so it can be written out by the DMA engine
void pixelsToDmaBuffer(struct pixel* pixelInput, uint8_t bufferOutput[][512]) {
  for(int row = 0; row < (LED_ROWS/ROWS_PER_OUTPUT); row++) {
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
        
        bufferOutput[depth][(row*LED_COLS + col)*2 + 0] = output;
        bufferOutput[depth][(row*LED_COLS + col)*2 + 1] = output | 1 << DMA_CLK_SHIFT;
      }
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
//  DMA_ERQ = 0;
  
  
  makeFadeCircle(Pixels, 16, 8);
  pixelsToDmaBuffer(Pixels, DmaBuffer[0]);
}

void writeTCD(uint8_t* source, int size) {
  // DMA channel #1 responsable for shifting out the data/clock stream
  DMA_TCD1_SADDR = source;                                        // Address to read from
  DMA_TCD1_SOFF = 1;                                              // Bytes to increment source register between writes 
  DMA_TCD1_ATTR = DMA_TCD_ATTR_SSIZE(0) | DMA_TCD_ATTR_DSIZE(0);  // 8-bit input and output
  DMA_TCD1_NBYTES_MLNO = size;                                    // Number of bytes to transfer in the minor loop
  DMA_TCD1_SLAST = 0;                                             // Bytes to add after a major iteration count (N/A)
  DMA_TCD1_DADDR = &GPIOC_PDOR;                                   // Address to write to
  DMA_TCD1_DOFF = 0;                                              // Bytes to increment destination register between write
  DMA_TCD1_CITER_ELINKNO = 1;                                     // Number of major loops to complete
  DMA_TCD1_BITER_ELINKNO = 1;                                     // Reset value for CITER (must be equal to CITER)
  DMA_TCD1_DLASTSGA = 0;                                          // Address of next TCD (N/A)
  
  DMA_TCD1_CSR |= DMA_TCD_CSR_START;                              // Start the transfer
  while (!(DMA_TCD1_CSR & DMA_TCD_CSR_DONE));                     // Wait for the transfer to complete
}


float animationX = 0;
float animationY = 0;
int counts = 0;
int frame = 0;

float xSpeed = .14;
float ySpeed = .18;

void loop() {
  // Now, start cycling through the rows
  for(int row = 0; row < (LED_ROWS/ROWS_PER_OUTPUT); row++) {

    // Set up the address outputs (todo: Delay this?)
    digitalWrite(LED_A, (row >> 0) & 0x01);
    digitalWrite(LED_B, (row >> 1) & 0x01);
    digitalWrite(LED_C, (row >> 2) & 0x01);
    
    for(int depth = 0; depth < BIT_DEPTH; depth++) {
      
//      // draw the current column
//      for(int col = 0; col < LED_COLS; col++) {
//        
//        GPIOC_PDOR = DmaBuffer[frame][depth][(row*LED_COLS + col)*2    ];
//        GPIOC_PDOR = DmaBuffer[frame][depth][(row*LED_COLS + col)*2 + 1];
//      }

      writeTCD(&DmaBuffer[frame][depth][row*LED_COLS*2], LED_COLS*2);

      digitalWrite(LED_STB, HIGH);
      digitalWrite(LED_STB, LOW);
  
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
