/** DMA_DAC_Render_PingPong
 * 
 *  Example demonstrating ping-pong (double) buffering of samples sent to the DAC,  
 *  configured using DMA. Output latency (time between rendering and outputting a 
 *  sample) is BUFFER_LEN/SAMPLE_RATE. 
 *  
 *  This example configures DMA channel 0 to transfer sample from dma_src_buffer
 *  to the DAC, triggered by PIT channel 0.
 */

#include "Oscillator.h"

#define BAUD_RATE (230400)
#define SAMPLE_RATE (48000.0f)
#define PIT_PERIOD ((uint16_t)(F_BUS / SAMPLE_RATE)-1)
#define BUFFER_LEN (512)
#define DAC_RES (12)
#define DAC_MAX ((1 << DAC_RES))
#define DAC_HALF ((1 << (DAC_RES-1)))

/** NOTE: For a ping-pong buffer configuration, we will double the length of the DMA source
 *  buffer, and maintain pointers to its first and second halves, which we'll call 'render' 
 *  and 'output'. While the DMA is sending samples from the output buffer to the DAC, we'll 
 *  use the CPU to fill the render buffer with samples from the oscillator. 
 *  
 *  We'll configure the DMA ISR to fire at the end and halfway through each major loop. Each 
 *  ISR call will then swap the render and output pointers.
 */
static volatile uint16_t dma_src_buffer[2*BUFFER_LEN];
static volatile uint16_t * render_buffer = &dma_src_buffer[0];
static volatile uint16_t * output_buffer = &dma_src_buffer[BUFFER_LEN];
Oscillator osc(SAMPLE_RATE, 440.0);

void setup() {
  
  Serial.begin(BAUD_RATE);

  // Initialize DMA source buffer
  for (int i = 0; i < 2*BUFFER_LEN; i++)
    dma_src_buffer[i] = DAC_HALF;

  init_dac0();
  init_dma0();
  init_pit0();
}

void init_dac0() {

  // (13.2.12)  System clock gating control register 2
  SIM_SCGC2 |= SIM_SCGC2_DAC0;    // Enable DAC0 clock

  // (41.5.4)   DAC Control Register 0 
  DAC0_C0 |= DAC_C0_DACEN;    // Enable DAC
  DAC0_C0 |= DAC_C0_DACRFS;   // Select DACREF_2 (3.3V)        
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
  DMA_TCD0_SADDR = (volatile uint16_t *)&dma_src_buffer;   
  
  // (24.3.20)  TCD Transfer Attributes 
  DMA_TCD0_ATTR = 0x0000;
  DMA_TCD0_ATTR |= DMA_TCD_ATTR_SSIZE(1);   // Source data transfer size (16-bit)   
  
  // (24.3.19)  TCD Signed Source Address Offset
  DMA_TCD0_SOFF = sizeof(uint16_t);         // Advance source pointer by 2 bytes per value       
  
  // (24.3.23)  TCD Signed Minor Loop Offset (Minor Loop Enabled and Offset Enabled)
  DMA_TCD0_NBYTES_MLOFFYES = sizeof(uint16_t);        // 2 bytes per minor loop
  DMA_TCD0_NBYTES_MLOFFYES |= DMA_TCD_NBYTES_SMLOE;   // Src. minor loop offset enable 
  DMA_TCD0_NBYTES_MLOFFYES &= ~DMA_TCD_NBYTES_DMLOE;  // Dest. minor loop offset disable 
  
  // TCD Current Minor Loop Link, Major Loop Count
  DMA_TCD0_BITER_ELINKNO = sizeof(dma_src_buffer) / sizeof(uint16_t);   // (24.3.32) Begining count
  DMA_TCD0_CITER_ELINKNO = sizeof(dma_src_buffer) / sizeof(uint16_t);   // (24.3.28) Current count
  
  // (24.3.24)  TCD Last Source Address Adjustment 
  DMA_TCD0_SLAST = -sizeof(dma_src_buffer);   // Return to &dma_src_buffer[0] after major loop   

  /* === Destination === */
  // (24.3.24)  TCD Destination Address 
  DMA_TCD0_DADDR = (volatile uint16_t *)&DAC0_DAT0L;

  // (24.3.20)  TCD Transfer Attributes 
  DMA_TCD0_ATTR |= DMA_TCD_ATTR_DSIZE(1);   // Destination data transfer size (16-bit)

  // (24.3.26)  TCD Signed Destination Address Offset 
  DMA_TCD0_DOFF = 0;

  // (24.3.29)  TCD Last Destination Address Adjustment/Scatter Gather Address 
  DMA_TCD0_DLASTSGA = 0;

  /* === Interrupts === */
  // (24.3.30)  TCD Control and Status 
  DMA_TCD0_CSR |= DMA_TCD_CSR_INTMAJOR;   // Interrupt at major loop  (CITER == 0)
  DMA_TCD0_CSR |= DMA_TCD_CSR_INTHALF;    // Also interrupt at half major loop (CITER == BITER/2)

  // (24.3.8)  Set Enable Request Register 
  DMA_SERQ |= DMA_SERQ_SERQ(0);           // Enable channel 0 DMA requests

  NVIC_ENABLE_IRQ(IRQ_DMA_CH0);   // Enable interrupt request line for DMA channel 0

  /* === Triggering === */  
  /** NOTE: Normally a DMA channel source peripheral (ADC, DAC, UART, etc.) triggers a DMA 
   *  request. DMA channels 0-3 are capable of Periodic Trigger mode, where one of the PIT channels 
   *  'gates' the DMA request, which must still come from a DMA channel source. So to configure 
   *  DMA transfers at a regular interval, we can either set the channel source to one of the 
   *  timer peripherals (FTM, or PDB), or we can set the channel source to one of ten 'always on' 
   *  sources and configure the DMA channel's associated PIT channel {PIT_CH0 -> DMA_CH0, ..., 
   *  PIT_CH3 -> DMA_CH3} to run at our desired sampling rate. 
    */
  // (23.4.1)   Channel Configuration Register 
  // - Configure DMA0 periodic trigger from PIT0
  DMAMUX0_CHCFG0 = DMAMUX_SOURCE_ALWAYS0;   // Trigger continuously
  DMAMUX0_CHCFG0 |= DMAMUX_TRIG;            // DMA channel periodic trigger enable
  DMAMUX0_CHCFG0 |= DMAMUX_ENABLE;          // DMA channel enable  
}

void loop() {}

void dma_ch0_isr(void) {

  // Swap pointers to output and render buffers
  static volatile uint16_t * tmp;
  tmp = output_buffer;
  output_buffer = render_buffer;
  render_buffer = tmp;

  /** NOTE: We have BUFFER_LEN/SAMPLE_RATE seconds to fill our buffer.
   *  
   *  Try inserting some delays below (before the lines filling the buffer). Any delay 
   *  longer than BUFFER_LEN/SAMPLE_RATE will cause glitches in the signal.
   */    
   
  // Fill DMA source buffer with samples from the oscillator, rescaled to [0, DAC_MAX]
  for (int i = 0; i < BUFFER_LEN; i++)
    render_buffer[i] = DAC_HALF * (1.0 + osc.render());
  
  DMA_CINT |= DMA_CINT_CINT(0);   // (24.3.12)  Clear interrupt request register
}

