/** DMA_ADC_Convert_PingPong
 *  
 *  Example demonstrating hardware trigger of ADC0 conversions using PIT0, with 
 *  ADC0 conversions configured to trigger DMA transfers of ADC0 results into a
 *  ping-pong (double) buffer. 
 *  
 *  NTOE: ADC's conversion time must be less than the period of the PIT to get  
 *        accurate sample rates. If the speicified sample rate isn't achieved:
 *        
 *        - raise the ADC clock rate using ADC_CFG1_ADIV
 *        - lower the number of samples used for hardware averaging with ADC_SC3_AVGS 
 *        - disable hardware averaging altogether         
 */
#define BAUD_RATE (230400)
#define SAMPLE_RATE (1000.0)
#define PIT_PERIOD ((uint16_t)(F_BUS / SAMPLE_RATE)-1)
#define BUFFER_LEN (8)

/** NOTE: For a ping-pong buffer configuration, we will double the length of the DMA dest
 *  buffer, and maintain pointers to its first and second halves, which we'll call 'input' 
 *  and 'proc'. While the DMA is transferring samples from the ADC to the input buffer, we'll 
 *  use the CPU to do any necessary processing on the proc buffer.
 *  
 *  We'll configure the DMA ISR to fire at the end and halfway through each major loop. Each 
 *  ISR call will then swap the input and proc pointers.
 */
static volatile uint16_t dma_dest_buffer[2*BUFFER_LEN];
static volatile uint16_t * input_buffer = &dma_dest_buffer[0];
static volatile uint16_t * proc_buffer = &dma_dest_buffer[BUFFER_LEN];

// See table 39.1.3.1 for pin mapping
int ch = 5;  // ADC0 channel 5 => AD4b, input signal ADC0_SE4b
// (Mapped to pin A0 on Teensy 3.6)

void setup() {
  Serial.begin(BAUD_RATE);
  init_adc0();
  init_pit0();
  init_dma0();
}

void init_adc0() {

  // (13.2.16)  System clock gating control register 6
  SIM_SCGC6 |= SIM_SCGC6_ADC0;        // Enable ADC0 clock  

  // (39.4.2)   ADC Configuration Register 1
  ADC0_CFG1 |= ADC_CFG1_ADICLK(0);    // Input clock = Bus clock
  ADC0_CFG1 |= ADC_CFG1_MODE(3);      // Single-ended, 16-bit
  ADC0_CFG1 &= ~ADC_CFG1_ADLSMP;      // Short sample time
  ADC0_CFG1 |= ADC_CFG1_ADIV(1);      // Input clock / 2
  ADC0_CFG1 &= ~ADC_CFG1_ADLPC;       // Normal power configuration
  // ADC Clock = F_BUS / 2 = 30MHz

  // (39.4.3)   ADC Configuration Register 2
  ADC0_CFG2 |= ADC_CFG2_ADLSTS(2);    // 6 extra ADCK cycles, 10 total sample time
  ADC0_CFG2 &= ~ADC_CFG2_ADHSC;       // Normal speed conversion
  ADC0_CFG2 |= ADC_CFG2_MUXSEL;       // ADxxB channels are selected (See table 39.1.3.1)

  // (39.4.6)   Status and Control Register 2
  ADC0_SC2 |= ADC_SC2_REFSEL(0);      // Voltage reference = Vcc
  ADC0_SC2 |= ADC_SC2_DMAEN;          // Enable DMA
  ADC0_SC2 &= ~ADC_SC2_ACREN;         // Disable comp. function range
  ADC0_SC2 &= ~ADC_SC2_ACFGT;         // Disable comp. function greater than
  ADC0_SC2 &= ~ADC_SC2_ACFE;          // Disable comp. function
  ADC0_SC2 &= ~ADC_SC2_ADTRG;         // Software trigger (for calibration)
   
  // (39.4.6)   Status and Control Register 3
  ADC0_SC3 |= ADC_SC3_AVGS(0);        // Hardware average 4 samples
  ADC0_SC3 |= ADC_SC3_AVGE;           // Enable hardware averaging
  ADC0_SC3 &= ~ADC_SC3_ADCO;          // Single conversion 

  // Run calibration
  adc_calibrate();

  // (39.4.6)   Status and Control Register 2
  ADC0_SC2 |= ADC_SC2_ADTRG;          // Enable hardware trigger

  // (13.2.7)   System options register 7
  SIM_SOPT7 |= SIM_SOPT7_ADC0ALTTRGEN;      // Alt trigger select
  SIM_SOPT7 |= SIM_SOPT7_ADC0TRGSEL(4);     // PIT trigger 0 select

  // (39.4.1)   ADC Status and Control Registers 1 (ADCx_SC1n)
  uint32_t sc1a_config;
  sc1a_config |= ADC_SC1_ADCH(ch);      // Channel selection
  sc1a_config &= ~ADC_SC1_DIFF;         // Single-ended mode
  sc1a_config &= ~ADC_SC1_AIEN;         // Disable interrupts
  ADC0_SC1A = sc1a_config;
} 

void adc_calibrate() {

  uint16_t sum;

  ADC0_SC3 |= ADC_SC3_CAL;    // Initiate automatic calibration sequence

  while (ADC0_SC3 & ADC_SC3_CAL) {/* wait for it... */}

  if (ADC0_SC3 & ADC_SC3_CALF)
    Serial.println("Calibration Failed");
  else
    Serial.println("Calibration Successful");

  // Plus-side gain calibration
  sum = (ADC0_CLP0 + ADC0_CLP1 + ADC0_CLP2 + ADC0_CLP3 + ADC0_CLP4 + ADC0_CLPS) >> 1;
  sum |= (1 << 15);
  ADC0_PG = sum;

  // Minus-side gain calibration
  sum = (ADC0_CLMS + ADC0_CLM4 + ADC0_CLM3 + ADC0_CLM2 + ADC0_CLM1 + ADC0_CLM0) >> 1;
  sum |= (1 << 15);
  ADC0_MG = sum;
}

void init_pit0() {
  SIM_SCGC6 |= SIM_SCGC6_PIT;         // (13.2.16)  Send system clock to PIT
  PIT_MCR = 0x00;                     // (46.4.1)   Turn on PIT                   
  PIT_LDVAL0 = PIT_PERIOD;            // (46.4.4)   Set timer 0 period
  PIT_TCTRL0 |= PIT_TCTRL_TEN;        // (46.4.6)   Enable timer 
}

void init_dma0() {

  /* === Initialize === */
  // (13.2.16-17)   System clock gating control registers 6 and 7
  SIM_SCGC6 |= SIM_SCGC6_DMAMUX;  // Enable DMAMUX clock
  SIM_SCGC7 |= SIM_SCGC7_DMA;     // Enable DMA clock
  
  // (24.3.1)   Control Register
  DMA_CR |= DMA_CR_EMLM;    // Enable minor loop mapping           

  /* === Source === */
  // (24.3.18)  TCD Source Address 
  DMA_TCD0_SADDR = (volatile uint16_t *)&ADC0_RA;   
  
  // (24.3.20)  TCD Transfer Attributes 
  DMA_TCD0_ATTR = 0x0000;
  DMA_TCD0_ATTR |= DMA_TCD_ATTR_SSIZE(1);   // Source data transfer size (16-bit)   
  
  // (24.3.19)  TCD Signed Source Address Offset
  DMA_TCD0_SOFF = 0;                        // Don't advance source pointer after minor loop      
  
  // (24.3.23)  TCD Signed Minor Loop Offset (Minor Loop Enabled and Offset Enabled)
  DMA_TCD0_NBYTES_MLOFFYES = sizeof(uint16_t);        // 2 bytes per minor loop
  DMA_TCD0_NBYTES_MLOFFYES &= ~DMA_TCD_NBYTES_SMLOE;  // Src. minor loop offset disable
  DMA_TCD0_NBYTES_MLOFFYES |= DMA_TCD_NBYTES_DMLOE;   // Dest. minor loop offset enable
  
  // TCD Current Minor Loop Link, Major Loop Count
  DMA_TCD0_BITER_ELINKNO = sizeof(dma_dest_buffer) / sizeof(uint16_t);   // (24.3.32) Begining count
  DMA_TCD0_CITER_ELINKNO = sizeof(dma_dest_buffer) / sizeof(uint16_t);   // (24.3.28) Current count
  
  // (24.3.24)  TCD Last Source Address Adjustment 
  DMA_TCD0_SLAST = 0;                         // Don't advance source pointer after major loop   

  /* === Destination === */
  // (21.3.24)  TCD Destination Address 
  DMA_TCD0_DADDR = (volatile uint16_t *)&dma_dest_buffer;

  // (24.3.20)  TCD Transfer Attributes 
  DMA_TCD0_ATTR |= DMA_TCD_ATTR_DSIZE(1);   // Destination data transfer size (16-bit)

  // (24.3.26)  TCD Signed Destination Address Offset 
  DMA_TCD0_DOFF = sizeof(uint16_t);         // Advance dest pointer by 2 bytes per value     

  // (24.3.29) TCD Last Destination Address Adjustment/Scatter Gather Address 
  DMA_TCD0_DLASTSGA = -sizeof(dma_dest_buffer);   // Return to &dma_dest_buffer[0] after major loop
    
  /* === Interrupts === */
  // (24.3.30)  TCD Control and Status
  DMA_TCD0_CSR |= DMA_TCD_CSR_INTMAJOR;   // Interrupt at major loop  (CITER == 0)
  DMA_TCD0_CSR |= DMA_TCD_CSR_INTHALF;    // Also interrupt at half major loop (CITER == BITER/2)

  // (24.3.8)  Set Enable Request Register 
  DMA_SERQ |= DMA_SERQ_SERQ(0);           // Enable channel 0 DMA requests

  NVIC_ENABLE_IRQ(IRQ_DMA_CH0);   // Enable interrupt request line for DMA channel 0

  /* === Triggering === */  
  // (23.4.1)   Channel Configuration Register 
  DMAMUX0_CHCFG0 = DMAMUX_SOURCE_ADC0;      // Trigger after ADC0 transfers
  DMAMUX0_CHCFG0 |= DMAMUX_ENABLE;          // DMA channel enable  
}

void loop() {}

void dma_ch0_isr(void) {

  // Swap pointers to input and proc buffers
  static volatile uint16_t * tmp;
  tmp = input_buffer;
  input_buffer = proc_buffer;
  proc_buffer = tmp;

  Serial.print("proc_buffer = [");
  for (int i = 0; i < BUFFER_LEN; i++) {
    Serial.print(proc_buffer[i]);
    Serial.print(" ");
  }
  Serial.println("]");
  
  DMA_CINT |= DMA_CINT_CINT(0);   // (24.3.12)  Clear interrupt request register
}

