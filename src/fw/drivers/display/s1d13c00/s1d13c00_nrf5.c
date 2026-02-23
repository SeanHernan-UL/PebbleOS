/*
 * Copyright 2025 Core Devices LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "s1d13c00.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "applib/graphics/gtypes.h"
#include "board/board.h"
#include "drivers/gpio.h"
#include "kernel/events.h"
#include "kernel/util/sleep.h"
#include "kernel/util/stop.h"
#include "os/mutex.h"
#include "system/passert.h"
#include <hal/nrf_gpio.h>
#include <hal/nrf_gpiote.h>
#include <hal/nrf_rtc.h>
#include <nrfx_gppi.h>
#include <nrfx_spim.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#define DISP_MODE_WRITE 0x01U
#define DISP_MODE_CLEAR 0x04U

static uint8_t s_buf[2 + ((DISP_LINE_BYTES + 2) * PBL_DISPLAY_HEIGHT)];
static bool s_updating;
static UpdateCompleteCallback s_uccb;
static SemaphoreHandle_t s_sem;

// s1d13c00 specific defines...
#define READ 0x03
#define FASTREAD 0x0B
#define PAGEPROG 0x02

#define RAM_START (0x20000000)
#define RAM_END (0x20017fff)

// System Control
#define SYSCTRL (0x400030e0)
#define RESET ((0x1) << 15)
#define IOSC_TRIM ((0x24) << 8) // original trim was 0x9s
#define INT_POL ((0x1) << 4)
#define IOSC_STABLE ((0x1) << 3) // (read only)
#define IOSC_8MHZ ((0x0) << 1)
#define IOSC_12MHZ ((0x1) << 1)
#define IOSC_16MHZ ((0x2) << 1)
#define IOSC_20MHZ ((0x3) << 1)
#define START_IOSC ((0x1) << 0)

// System Interrupts Status
#define SYSINTS (0x400030e4)

// System Protection
#define SYSPROT (0x40000000)
#define PROT_EN (0x0000)
#define PROT_DIS (0x0096)

// Oscillation Control
#define CLGOSC (0x40000042)
#define OSC1_EN ((0x1) << 1)
#define OSC1_DIS ((0x0) << 1)

// OSC1 Control
#define CLGOSC1 (0x40000046)
#define OSDRB ((0x1) << 14)
#define OSDEN ((0x1) << 13)
#define OSC1BUP ((0x1) << 12)
#define OSC1SELCR_EXT ((0x0) << 11)
#define OSC1SELCR_IN ((0x1) << 11)
#define INVIB ((0x2) << 6)
#define INVIN ((0x1) << 4)
#define OSC1WT ((0x2) << 0)

// CLG Interrupt flag
#define CLGINTF (0x4000004c)
#define OSC1STAIF ((0x1) << 1)
#define OSC1STPIF ((0x1) << 5)

// CLG Interrupt flag
#define CLGINTE (0x4000004e)
#define OSC1STAIE ((0x1) << 1)
#define OSC1STPIE ((0x1) << 5)

// OSC1 Internal Trimming
#define GCLGOSC1TRM (0x40000054)
#define OSC1_TRIM (0x0022)

// MDC Voltage Booster/Regulator clock control
#define MDCBSTCLK (0x40003080)
#define SCRATCHPAD_BIT0 ((0x1) << 7)
#define CLKDIV ((0x5) << 4)
#define CLKSRC_OSC1 ((0x1) << 0)
#define CLKSRC_IOSC ((0x0) << 0)

// MDC Power Output control
#define MDCBSTPWR (0x40003084)
#define VMDBUP ((0x1) << 3)
#define BSTON ((0x1) << 2)
#define REGECO ((0x1) << 1)
#define REGON ((0x1) << 0)

// MDC Voltage Booster/Regulator VMD Output Control
#define MDCBSTVMD (0x40003088)
#define VMDHVOL_5V0 ((0x7) << 12)
#define VMDHVOL_4V5 ((0x2) << 12)
#define VMDHON ((0x1) << 8)
#define VMDLVOL_3V2 ((0x4) << 4)
#define VMDLON ((0x1) << 0)


// MDC Display Control
#define MDCDISPCTL (0x40003000)
#define DISPGS_0 ((0x0) << 11)
#define DISPINVERT ((0x1) << 7)
#define DISPSPI_0 ((0x0) << 4)
#define ROTSEL_270 ((0x3) << 2)
#define ROTSEL_180 ((0x2) << 2)
#define ROTSEL_90 ((0x1) << 2)
#define ROTSEL_NONE ((0x0) << 2)
#define VCOMEN ((0x1) << 1)
#define DISPEPD_0 ((0x0) << 0)

// MDC Display Width
#define MDCDISPWIDTH (0x40003002)

// MDC Display height
#define MDCDISPHEIGHT (0x40003004)

// MDC VCOM clock divider
#define MDCDISPVCOMDIV (0x40003006)

// MDC Display clock divider
#define MDCDISPCLKDIV (0x40003008)

// MDC Display paramters 1 and 2
#define MDCDISPPRM21 (0x4000300a)

// MDC Display paramters 3 and 4
#define MDCDISPPRM43 (0x4000300c)

// MDC Display paramters 5 and 6
#define MDCDISPPRM65 (0x4000300e)

// MDC Display paramters 7 and 8
#define MDCDISPPRM87 (0x40003010)

// MDC Display update start line
#define MDCDISPSTARTY (0x40003012)

// MDC Display update start line
#define MDCDISPENDY (0x40003014)

// MDC Display stride
#define MDCDISPSTRIDE (0x40003016)

// MDC Display frame buffer base address 0
#define MDCDISPFRMBUFF0 (0x40003018)

// MDC Display frame buffer base address 1
#define MDCDISPFRMBUFF1 (0x4000301a)

// MDC Trigger control
#define MDCTRIGCTL (0x4000301c)
#define UPDTRIG ((0x1) << 1)
#define GFXTRIG ((0x1) << 0)

// MDC Interrupt control
#define MDCINTCTL (0x4000301e)
#define VCNTIE ((0x1) << 10)
#define UPDIE ((0x1) << 9)
#define GFXIE ((0x10) << 8)
#define VCNTIF ((0x1) << 2)
#define UPDIF ((0x1) << 1)
#define GFXIF ((0x1) << 1)

// MDC Graphics control
#define MDCGFXCTL (0x40003020)

// MDC Input X coordinate
#define MDCGFXIXCENTER (0x40003022)

// MDC Input Y coordinate
#define MDCGFXIYCENTER (0x40003024)

// MDC Input width
#define MDCGFXIWIDTH (0x40003026)

// MDC Input height
#define MDCGFXIHEIGHT (0x40003028)

// MDC Output X coordinate
#define MDCGFXOXCENTER (0x4000302a)

// MDC Output Y coordinate
#define MDCGFXOYCENTER (0x4000302c)

// MDC Output width
#define MDCGFXOWIDTH (0x4000302e)

// MDC Output height
#define MDCGFXOHEIGHT (0x40003030)

// MDC Input height
#define MDCGFXIHEIGHT (0x40003028)

// MDC X Left scale
#define MDCGFXXLSCALE (0x40003032)

// MDC X Right scale
#define MDCGFXXRSCALE (0x40003034)

// MDC Y Top scale
#define MDCGFXXYTSCALE (0x40003036)

// MDC Y Bottom scale
#define MDCGFXXYBSCALE (0x40003038)

// MDC X/Y Shear
#define MDCGFXSHEAR (0x4000303a)

// MDC Rotation
#define MDCGFXROTVAL (0x4000303c)

// MDC Colour
#define MDCGFXCOLOR (0x4000303e)

// MDC Source window base address 0
#define MDCGFXIBADDR0 (0x40003040)

// MDC Source window base address 1
#define MDCGFXIBADDR1 (0x40003042)

// MDC Destination window base address 0
#define MDCGFXOBADDR0 (0x40003044)

// MDC Destination window base address 1
#define MDCGFXOBADDR1 (0x40003046)

// MDC Source Image stride
#define MDCGFXISTRIDE (0x40003048)

// MDC Destination Image stride
#define MDCGFXOSTRIDE (0x4000304a)

// MDC Output window left edge
#define MDCGFXOWLEFT (0x4000304c)

// MDC Output window right edge
#define MDCGFXOWRIGHT (0x4000304e)

// MDC Output window top edge
#define MDCGFXOWTOP (0x40003050)

// MDC Output window bottom edge
#define MDCGFXOWBOT (0x40003052)

// MDC Display paramters 9 and 10
#define MDCDISPPRM109 (0x40003054)

// MDC Display paramters 11 and 12
#define MDCDISPPRM1211 (0x40003056)

// MDC Display paramters 13 and 14
#define MDCDISPPRM1413 (0x40003058)

// MDC Display control 2
#define MDCDISPCTL2 (0x4000305a)
#define GSALPHA ((0x1) << 5)
#define CSPOL ((0x1) << 4)
#define VSTFALL ((0x1) << 3)
#define HSTFALL ((0x1) << 2)
#define ENBPHASE ((0x1) << 1)
#define FASTVCK ((0x1) << 0)

// MDC VCK count compare
#define MDCVCNTCOMP (0x4000305c)

// MDC VCK count
#define MDCVCNT (0x4000305e)


// MCD VCOM Clock control register
#define MDCVCOMCLKCTL (0x40003068)

// MDC Voltage Booster/regulator VMD output control
#define MDCBSTTVMD (0x40003088)


static uint8_t out_buf[32];
static uint8_t in_buf[32];

void prv_display_clear(uint8_t colour);
void prv_update_command();
uint16_t prv_read(unsigned long address);
void prv_write(unsigned long address, uint16_t value);

// static void prv_extcomin_init(void) {
//   nrfx_err_t err;
//   const NrfLowPowerPWM *extcomin = &BOARD_CONFIG_DISPLAY.extcomin;
//   uint32_t evt_addr, task_addr;
//   uint8_t ppi_ch[2];

//   nrf_gpiote_te_default(extcomin->gpiote, extcomin->gpiote_ch);

//   nrf_gpio_pin_write(extcomin->psel, 0);
//   nrf_gpio_cfg_output(extcomin->psel);

//   // RTC: CC0 is the period end, CC1 is the pulse end
//   nrf_rtc_task_trigger(extcomin->rtc, NRF_RTC_TASK_STOP);
//   nrf_rtc_event_clear(extcomin->rtc, nrf_rtc_compare_event_get(0));
//   nrf_rtc_event_clear(extcomin->rtc, nrf_rtc_compare_event_get(1));
//   nrf_rtc_task_trigger(extcomin->rtc, NRF_RTC_TASK_CLEAR);
//   nrf_rtc_prescaler_set(extcomin->rtc, NRF_RTC_FREQ_TO_PRESCALER(32768));
//   nrf_rtc_event_enable(extcomin->rtc, (NRF_RTC_INT_COMPARE0_MASK | NRF_RTC_INT_COMPARE1_MASK));
//   nrf_rtc_cc_set(extcomin->rtc, 0, (32768 * extcomin->period_us) / 1000000 - 1);
//   nrf_rtc_cc_set(extcomin->rtc, 1, (32768 * extcomin->pulse_us) / 1000000 - 1);

//   nrf_gpiote_task_configure(extcomin->gpiote, extcomin->gpiote_ch, extcomin->psel,
//                             NRF_GPIOTE_POLARITY_NONE, NRF_GPIOTE_INITIAL_VALUE_LOW);
//   nrf_gpiote_task_enable(extcomin->gpiote, extcomin->gpiote_ch);

//   err = nrfx_gppi_channel_alloc(&ppi_ch[0]);
//   PBL_ASSERTN(err == NRFX_SUCCESS);

//   err = nrfx_gppi_channel_alloc(&ppi_ch[1]);
//   PBL_ASSERTN(err == NRFX_SUCCESS);

//   // Period end (CC0) sets GPIO, clears RTC
//   evt_addr = nrf_rtc_event_address_get(extcomin->rtc, nrf_rtc_compare_event_get(0));
//   task_addr =
//       nrf_gpiote_task_address_get(extcomin->gpiote, nrf_gpiote_set_task_get(extcomin->gpiote_ch));
//   nrfx_gppi_channel_endpoints_setup(ppi_ch[0], evt_addr, task_addr);

//   task_addr = nrf_rtc_event_address_get(extcomin->rtc, NRF_RTC_TASK_CLEAR);
//   nrfx_gppi_fork_endpoint_setup(ppi_ch[0], task_addr);

//   // Pulse end (CC1) clears GPIO
//   evt_addr = nrf_rtc_event_address_get(extcomin->rtc, nrf_rtc_compare_event_get(1));
//   task_addr =
//       nrf_gpiote_task_address_get(extcomin->gpiote, nrf_gpiote_clr_task_get(extcomin->gpiote_ch));
//   nrfx_gppi_channel_endpoints_setup(ppi_ch[1], evt_addr, task_addr);

//   nrfx_gppi_channels_enable((1UL << ppi_ch[0]) | (1UL << ppi_ch[1]));

//   nrf_rtc_task_trigger(extcomin->rtc, NRF_RTC_TASK_START);
// }

static inline void prv_enable_spim(void) {
  nrf_spim_enable(BOARD_CONFIG_DISPLAY.spi.p_reg);
}

static inline void prv_disable_spim(void) {
  nrf_spim_disable(BOARD_CONFIG_DISPLAY.spi.p_reg);

  // Workaround for nRF52840 anomaly 195
  // if (BOARD_CONFIG_DISPLAY.spi.p_reg == NRF_SPIM3) {
  //   *(volatile uint32_t *)0x4002F004 = 1;
  // }
}

static inline void prv_enable_chip_select(void) {
  gpio_output_set(&BOARD_CONFIG_DISPLAY.cs, false);
}

static inline void prv_disable_chip_select(void) {
  gpio_output_set(&BOARD_CONFIG_DISPLAY.cs, true);
}

static void prv_terminate_transfer(void *data) {
  s_updating = false;

  prv_disable_chip_select();
  prv_disable_spim();

  s_uccb();
}

static void prv_spim_evt_handler(nrfx_spim_evt_t const *evt, void *ctx) {
  portBASE_TYPE woken = pdFALSE;

  prv_disable_chip_select(); // temporary...

  if (s_updating) {
    PebbleEvent e = {
        .type = PEBBLE_CALLBACK_EVENT,
        .callback =
            {
                .callback = prv_terminate_transfer,
            },
    };

    woken = event_put_isr(&e) ? pdTRUE : pdFALSE;
  } else {
    xSemaphoreGiveFromISR(s_sem, &woken);
  }

  portEND_SWITCHING_ISR(woken);
}

void display_init(void) {
  
  // ensure the voltage regulator is turned on by pulling pin 0.15 low...
  gpio_output_init(&BOARD_CONFIG_DISPLAY.on_ctrl, GPIO_OType_PP, GPIO_Speed_50MHz);
  gpio_output_set(&BOARD_CONFIG_DISPLAY.on_ctrl, false);

  nrfx_spim_config_t config = NRFX_SPIM_DEFAULT_CONFIG(
      BOARD_CONFIG_DISPLAY.clk.gpio_pin, BOARD_CONFIG_DISPLAY.mosi.gpio_pin,
      BOARD_CONFIG_DISPLAY.miso.gpio_pin, NRF_SPIM_PIN_NOT_CONNECTED);
  config.frequency = NRFX_MHZ_TO_HZ(.5);
  config.bit_order = NRF_SPIM_BIT_ORDER_MSB_FIRST;

  nrfx_err_t err = nrfx_spim_init(&BOARD_CONFIG_DISPLAY.spi, &config, prv_spim_evt_handler, NULL);
  PBL_ASSERTN(err == NRFX_SUCCESS);

  gpio_output_init(&BOARD_CONFIG_DISPLAY.cs, GPIO_OType_PP, GPIO_Speed_50MHz);
  gpio_output_set(&BOARD_CONFIG_DISPLAY.cs, true);

  // gpio_output_init(&BOARD_CONFIG_DISPLAY.on_ctrl, BOARD_CONFIG_DISPLAY.on_ctrl_otype,
  //                  GPIO_Speed_50MHz);
  // gpio_output_set(&BOARD_CONFIG_DISPLAY.on_ctrl, true);

  // prv_extcomin_init();

  s_sem = xSemaphoreCreateBinary();

  uint8_t stable = 0;

  // soft reset
  prv_write(SYSCTRL, RESET); 
  psleep(1); // sleep 1 ms

  // start IOSC
  prv_write(SYSCTRL, IOSC_TRIM+INT_POL+IOSC_20MHZ); // active low interrupt
  prv_write(SYSCTRL, IOSC_TRIM+INT_POL+IOSC_20MHZ+START_IOSC); // active low interrupt
  psleep(1); // sleep 1 ms
  prv_read(SYSCTRL);

  // disable FOUT pin...
  prv_write(0x40000212, 0x0); // turn GPIO in/out off
  prv_write(0x4000021c, 0x0); // turn peripheral mode off

  // enable OSC1 sequence
  prv_write(CLGINTF, OSC1STAIF); // clear int flag
  prv_write(CLGINTE, OSC1STAIE); // enable interrupt
  prv_write(SYSPROT, PROT_DIS); // disable write protect
  prv_write(CLGOSC1, OSDEN+OSC1SELCR_IN+INVIB+INVIN+OSC1WT);
  prv_write(SYSPROT, PROT_DIS); // disable write protect
  prv_write(CLGOSC, OSC1_EN); // start the OSC1
  psleep(1000); // sleep 1 s... FIXME: see if this time can be reduced later
  
  // check OSC1 stability
  prv_read(SYSINTS); // bit 6 (CLG) should be set
  prv_read(CLGINTF); // bit 1 should be set
  prv_write(CLGINTF, OSC1STAIF); // clear int flag
  prv_read(SYSINTS); // should be clear
  psleep(1); // sleep 1 ms

  // ## enable FOUT pin...
  prv_write(0x4000021e, ((0x1) << 2)); // select FOUT
  prv_write(0x4000021c, 0x2); // turn peripheral mode on
  prv_write(0x40000050, ((0x4) << 4)+((0x0) << 2)); // config FOUT
  prv_write(0x40000050, ((0x4) << 4)+((0x0) << 2)+((0x1) << 0)); // enable FOUT
  // ## measuring the internal OSC1 as 30.273 kHz... (adjust trim...)

  // power on of voltage supplies
  prv_write(MDCBSTCLK, CLKSRC_OSC1);
  prv_write(MDCBSTPWR, BSTON+REGECO+REGON);
  psleep(2); // sleep 2 ms

  // turn on VMDL (3.2 V)
  prv_write(MDCBSTVMD, VMDLVOL_3V2+VMDLON);
  psleep(2); // sleep 2 ms

  // turn on VMDH (5 V)
  prv_write(MDCBSTVMD, VMDHVOL_5V0+VMDHON+VMDLVOL_3V2+VMDLON);
  psleep(2); // sleep 2 ms

  // let display power up...
  psleep(1); // sleep 1 ms
  
  // display config (Sharp LS014B7DD01) (16 MHz clock as 20 MHz wouldn't work)
  prv_write(MDCDISPVCOMDIV, 266); // VCOM freq=32000/(4*(273+1))=~30Hz... Transmissive mode
  prv_write(MDCDISPCTL, DISPGS_0+DISPINVERT+DISPSPI_0+ROTSEL_NONE+DISPEPD_0); // review + #define...
  // prv_write(MDCDISPCTL, DISPGS_0+DISPINVERT+DISPSPI_0+ROTSEL_NONE+VCOMEN+DISPEPD_0);
  prv_write(MDCDISPCTL2, 0x0);
  prv_write(MDCDISPCLKDIV, ((0) << 8)+((3) << 1)); // t0 = tsGCK2 = (6+1) T = 350 ns, T = (1)/(20 MHz) ~= 187.5 ns
  prv_write(MDCDISPPRM21, ((0) << 8)+((145) << 0)); // t2 = thsBSP = (6+1) T = 350 ns, t1 = thsGSP = 49 us
  prv_write(MDCDISPPRM43, ((35) << 8)+((0) << 0));  // t4 = [TIM4*(t3+t7) + t0 + t2] = tsGCK1 = 20 us -> TIM4 =33, t3 = tsRGB = (6+1) T = 350 ns
  prv_write(MDCDISPPRM65, ((72) << 8)+((50) << 0));  // t6 = thsINTB = 25 us, t5 = [TIM5*(t3+t7)] = thwGEN = 30 us -> TIM5 = 50
  prv_write(MDCDISPPRM87, ((0) << 8)+((0) << 0)); // t8 = thsBSP = (6+1) T = 350 ns, t7 = thRGB = (6+1) T = 350 ns
  prv_write(MDCDISPPRM109, ((2) << 8)+((2) << 0)); // t9 = 100???, t10 = 100??? FIXME
  prv_write(MDCDISPPRM1211, ((3) << 8)+((2) << 0)); // t12 = 100???, t11 = 100??? FIXME
  prv_write(MDCDISPPRM1413, ((0) << 8)+((19) << 0)); // ----, t13 = thwGCK = 1 us (Fast forward GCK)
  prv_write(MDCDISPWIDTH, 280);
  prv_write(MDCDISPHEIGHT, 280);
  prv_write(MDCDISPFRMBUFF0, RAM_START & 0xffff);
  prv_write(MDCDISPFRMBUFF1, (RAM_START >> 16) & 0xffff); // 280*280 = 78400 = 0x00013240
  prv_write(MDCDISPSTRIDE, 280); // number of bytes in each horizontal row FIXME
  prv_write(MDCDISPSTARTY, 0);
  prv_write(MDCDISPENDY, 279);

  // ## Config calculations for 20 MHz
  // prv_write(MDCDISPVCOMDIV, 266); // VCOM freq=32000/(4*(273+1))=~30Hz... Transmissive mode
  // prv_write(MDCDISPCTL, DISPGS_0+DISPINVERT+DISPSPI_0+ROTSEL_NONE+VCOMEN+DISPEPD_0); // review + #define...
  // prv_write(MDCDISPCTL2, VSTFALL+HSTFALL+ENBPHASE);
  // prv_write(MDCDISPCLKDIV, ((2) << 8)+((2) << 1)); // t0 = tsGCK2 = (6+1) T = 350 ns, T = (1)/(20 MHz) = 50 ns
  // prv_write(MDCDISPPRM21, ((2) << 8)+((167) << 0)); // t2 = thsBSP = (6+1) T = 350 ns, t1 = thsGSP = 49 us
  // prv_write(MDCDISPPRM43, ((33) << 8)+((2) << 0));  // t4 = [TIM4*(t3+t7) + t0 + t2] = tsGCK1 = 20 us -> TIM4 =33, t3 = tsRGB = (6+1) T = 350 ns
  // prv_write(MDCDISPPRM65, ((83) << 8)+((50) << 0));  // t6 = thsXRST = 25 us, t5 = [TIM5*(t3+t7)] = thwGEN = 30 us -> TIM5 = 50
  // prv_write(MDCDISPPRM87, ((2) << 8)+((2) << 0)); // t8 = thsBSP = (6+1) T = 350 ns, t7 = thRGB = (6+1) T = 350 ns
  // prv_write(MDCDISPPRM109, ((100) << 8)+((100) << 0)); // t9 = 100???, t10 = 100??? FIXME
  // prv_write(MDCDISPPRM1211, ((100) << 8)+((100) << 0)); // t12 = 100???, t11 = 100??? FIXME
  // prv_write(MDCDISPPRM1413, ((0) << 8)+((19) << 0)); // ----, t13 = thwGCK = 1 us (Fast forward GCK)
  // prv_write(MDCDISPWIDTH, PBL_DISPLAY_WIDTH);
  // prv_write(MDCDISPHEIGHT, PBL_DISPLAY_HEIGHT);
  // prv_write(MDCDISPFRMBUFF0, RAM_START & 0xffff);
  // prv_write(MDCDISPFRMBUFF1, (RAM_START >> 16) & 0xffff); // 280*280 = 78400 = 0x00013240
  // prv_write(MDCDISPSTRIDE, PBL_DISPLAY_WIDTH); // number of bytes in each horizontal row FIXME
  // prv_write(MDCDISPSTARTY, 0);
  // prv_write(MDCDISPENDY, 279);
  
  // graphics config for clearing display
  prv_write(MDCGFXOBADDR0, RAM_START & 0xffff);
  prv_write(MDCGFXOBADDR1, (RAM_START >> 16) & 0xffff); 
  prv_write(MDCGFXOWIDTH, 280);
  prv_write(MDCGFXOHEIGHT, 280);
  prv_write(MDCGFXOSTRIDE, 280);
  
  // wipe screen
  display_clear();
  prv_update_command();

  psleep(1);
  
  // enable VCOM, VB, VA
  prv_write(MDCDISPCTL, DISPGS_0+DISPINVERT+DISPSPI_0+ROTSEL_NONE+VCOMEN+DISPEPD_0);

  // print screen purple
  prv_display_clear(0xf3);
  prv_update_command();

  stable = prv_read(SYSCTRL) & IOSC_STABLE;
  if(stable == 0) {
    psleep(1);
    stable = prv_read(SYSCTRL) & IOSC_STABLE;
    prv_read(SYSINTS);
  }
}

void display_clear() {

  PBL_ASSERTN(!s_updating);

  // config
  prv_write(MDCGFXIXCENTER, 0x0);
  prv_write(MDCGFXIYCENTER, 0x0);
  prv_write(MDCGFXOXCENTER, 280-1);
  prv_write(MDCGFXOYCENTER, 280-1);
  prv_write(MDCGFXCOLOR, 0xC0); // colour
  prv_write(MDCGFXIWIDTH, 0x0);
  prv_write(MDCGFXIHEIGHT, 0x0);
  prv_write(MDCGFXCTL, (1<<11));

  uint16_t intctl = prv_read(MDCINTCTL);
  prv_write(MDCINTCTL, intctl|(GFXIE+GFXIF)); // clear interrupt

  uint16_t gfxctl = prv_read(MDCGFXCTL);
  prv_write(MDCGFXCTL, gfxctl|0x2); // draw rectangle

  uint16_t trigctl = prv_read(MDCTRIGCTL);
  prv_write(MDCTRIGCTL, trigctl|(GFXTRIG));

  // wait for GXFTRIG bit to be cleared...
  // ## wait for up to 250 ms for GFXTRIG to go to 0
  for(uint8_t i=0; i<25; i++) {
    trigctl = prv_read(GFXTRIG);

    if ((trigctl && UPDTRIG) == 0) {
      break; // display update finished...
    } else {
      psleep(10);
    }
  }

  // cleanup
  intctl = prv_read(MDCINTCTL);
  prv_write(MDCINTCTL, intctl|(GFXIE+GFXIF)); // clear interrupt
}

void prv_display_clear(uint8_t colour) {

  PBL_ASSERTN(!s_updating);

  // config
  prv_write(MDCGFXIXCENTER, 0x0);
  prv_write(MDCGFXIYCENTER, 0x0);
  prv_write(MDCGFXOXCENTER, 280-1);
  prv_write(MDCGFXOYCENTER, 280-1);
  prv_write(MDCGFXCOLOR, colour); // colour
  prv_write(MDCGFXIWIDTH, 0x0);
  prv_write(MDCGFXIHEIGHT, 0x0);
  prv_write(MDCGFXCTL, (1<<11));

  uint16_t intctl = prv_read(MDCINTCTL);
  prv_write(MDCINTCTL, intctl|(GFXIE+GFXIF)); // clear interrupt

  uint16_t gfxctl = prv_read(MDCGFXCTL);
  prv_write(MDCGFXCTL, gfxctl|0x2); // draw rectangle

  uint16_t trigctl = prv_read(MDCTRIGCTL);
  prv_write(MDCTRIGCTL, trigctl|(GFXTRIG));

  // wait for GXFTRIG bit to be cleared...
  // ## wait for up to 250 ms for GFXTRIG to go to 0
  for(uint8_t i=0; i<25; i++) {
    trigctl = prv_read(GFXTRIG);

    if ((trigctl && UPDTRIG) == 0) {
      break; // display update finished...
    } else {
      psleep(10);
    }
  }

  // cleanup
  intctl = prv_read(MDCINTCTL);
  prv_write(MDCINTCTL, intctl|(GFXIE+GFXIF)); // clear interrupt
}

void display_set_enabled(bool enabled) {
  // gpio_output_set(&BOARD_CONFIG_DISPLAY.on_ctrl, enabled);
}

void prv_update_command() {
  // ## Tell the driver IC to update the display

  uint16_t pwr = prv_read(MDCBSTPWR);
  prv_write(MDCBSTPWR, (VMDBUP+BSTON+REGON)); // speed up VMD response

  // TODO: implement a modify register func...
  uint16_t intctl = prv_read(MDCINTCTL);
  prv_write(MDCINTCTL, intctl|(UPDIE+UPDIF)); // clear interrupt

  uint16_t trig = prv_read(MDCTRIGCTL);
  prv_write(MDCTRIGCTL, trig|(UPDTRIG));

  // ## wait for up to 250 ms for trig to go to 0
  for(uint8_t i=0; i<25; i++) {
    trig = prv_read(MDCTRIGCTL);

    if ((trig && UPDTRIG) == 0) {
      break; // display update finished...
    } else {
      psleep(10);
    }
  }

  // ## cleanup
  intctl = prv_read(MDCINTCTL);
  prv_write(MDCINTCTL, intctl|(UPDIE+UPDIF)); // clear interrupt

  pwr = prv_read(MDCBSTPWR);
  prv_write(MDCBSTPWR, (BSTON+REGECO+REGON)); // turn on economy mode

}

void display_update(NextRowCallback nrcb, UpdateCompleteCallback uccb) {
  DisplayRow row;
  uint8_t *pbuf = s_buf;
  nrfx_spim_xfer_desc_t desc = {.p_tx_buffer = pbuf};

  PBL_ASSERTN(!s_updating);

  // ## Fill the memory buffer on driver IC
  // write command (write)
  *pbuf++ = PAGEPROG;
  *pbuf++ = (RAM_START >> 24) & 0xff;
  *pbuf++ = (RAM_START >> 16) & 0xff;
  *pbuf++ = (RAM_START >> 8)  & 0xff;
  *pbuf++ = RAM_START & 0xff;
  desc.tx_length += 5; // 1 command byte + 4 address bytes

  while (nrcb(&row)) {
    // write row address, data and trailing dummy
    // *pbuf++ = row.address + 1;
    memcpy(pbuf, row.data, DISP_LINE_BYTES);
    // pbuf += DISP_LINE_BYTES;
    // *pbuf++ = 0x00;

    // desc.tx_length += DISP_LINE_BYTES + 2;
    desc.tx_length += DISP_LINE_BYTES;
  }

  // write last trailing dummy
  // *pbuf++ = 0x00;
  desc.tx_length++;

  prv_read(SYSCTRL);
  prv_read(SYSINTS);

  prv_enable_spim();
  prv_disable_chip_select();
  prv_enable_chip_select();
  
  // s_uccb = uccb;
  // s_updating = true;
  nrfx_err_t err = nrfx_spim_xfer(&BOARD_CONFIG_DISPLAY.spi, &desc, 0);
  PBL_ASSERTN(err == NRFX_SUCCESS);
  xSemaphoreTake(s_sem, portMAX_DELAY);

  prv_disable_chip_select();
  prv_disable_spim();

  prv_update_command();
}

bool display_update_in_progress(void) {
  return s_updating;
}

// (cmd[7:0]) (addr[31:0]) (data[7:0])...

// 16 bit addressing/registers
uint16_t prv_read(unsigned long address) {

  size_t len = 8;
  uint8_t *bob = out_buf;
  uint8_t const *out = &out_buf[0]; // making sure we are pointing to the start of the buffer...
  uint8_t *in = in_buf;

  // build the frame
  *bob++ = READ; // cmd
  *bob++ = (address >> 24) & 0xff;
  *bob++ = (address >> 16) & 0xff;
  *bob++ = (address >> 8)  & 0xff;
  *bob++ = address & 0xff;
  *bob++ = 0x00; // 1 byte dummy read
  *bob++ = 0x00; // 1 byte actual read (still dummy data)
  *bob++ = 0x00; //1 byte actual read (still dummy data)
  // total read of 2 bytes (16 bit reg)

  nrfx_spim_xfer_desc_t xfer;
  xfer.p_tx_buffer = out;
  xfer.tx_length = len;
  xfer.p_rx_buffer = in;
  xfer.rx_length = len;

  prv_enable_spim();
  prv_enable_chip_select();
  
  nrfx_err_t rv = nrfx_spim_xfer(&BOARD_CONFIG_DISPLAY.spi, &xfer, 0);
  PBL_ASSERTN(rv == NRFX_SUCCESS);
  xSemaphoreTake(s_sem, portMAX_DELAY);

  prv_disable_chip_select();
  prv_disable_spim();

  return ((in[7] << 8) + (in[6]));
}


// TODO: burst write... don't have time now :(
void prv_write(unsigned long address, uint16_t value) {

  size_t len = 7;
  uint8_t *bob = out_buf;
  uint8_t const *out = &out_buf[0]; // making sure we are pointing to the start of the buffer...
  uint8_t *in = in_buf;

  // build the frame
  *bob++ = PAGEPROG; // cmd
  *bob++ = (address >> 24) & 0xff;
  *bob++ = (address >> 16) & 0xff;
  *bob++ = (address >> 8)  & 0xff;
  *bob++ = address & 0xff;
  *bob++ = value & 0xff;
  *bob++ = (value >> 8) & 0xff;

  nrfx_spim_xfer_desc_t xfer;
  xfer.p_tx_buffer = out;
  xfer.tx_length = len;
  xfer.p_rx_buffer = in;
  xfer.rx_length = len;

  prv_enable_spim();
  prv_enable_chip_select();

  nrfx_err_t rv = nrfx_spim_xfer(&BOARD_CONFIG_DISPLAY.spi, &xfer, 0);
  PBL_ASSERTN(rv == NRFX_SUCCESS);
  xSemaphoreTake(s_sem, portMAX_DELAY);
  
  prv_disable_chip_select();
  prv_disable_spim();
}

/* stubs */

uint32_t display_baud_rate_change(uint32_t new_frequency_hz) {
  return new_frequency_hz;
}

void display_pulse_vcom(void) {}

void display_show_splash_screen(void) {}

void display_set_offset(GPoint offset) {}

GPoint display_get_offset(void) {
  return GPointZero;
}
