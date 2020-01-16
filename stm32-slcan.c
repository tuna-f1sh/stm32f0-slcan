/* ----------------------------------------------------------------------------
 * "THE BEER-WARE LICENSE" (Revision 42):
 * <info@gerhard-bertelsmann.de> wrote this file. As long as you retain this
 * notice you can do whatever you want with this stuff. If we meet some day,
 * and you think this stuff is worth it, you can buy me a beer in return
 * Gerhard Bertelsmann
 * ----------------------------------------------------------------------------
 *  Ported to STMF0 (Cortex M0) by J.Whittington 2019
 */

#include "stm32-slcan.h"

extern struct ring output_ring;
extern struct ring input_ring;
volatile unsigned int counter;
volatile uint8_t status;
volatile uint8_t commands_pending;
volatile bool connected = false;
uint8_t d_data[8];

static void gpio_setup(void) {
  /* A2 & A3 USART */
  rcc_periph_clock_enable(RCC_GPIOA);
  /* B8 & B9 CAN */
  rcc_periph_clock_enable(RCC_GPIOB);

  // Setup LEDs
  gpio_set(GPIOB, GPIO0);
  gpio_clear(GPIOB, GPIO1);
  gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO0);
  gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO1);

  /* Enable clocks for GPIO port A (for GPIO_USART2_TX) and USART2. */
  rcc_periph_clock_enable(RCC_USART2);
}

static void systick_setup(void) {
  systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
  /* clear counter so it starts right away */
  STK_CVR = 0;

  systick_set_reload(rcc_ahb_frequency / 1000);
  systick_counter_enable();
  systick_interrupt_enable();
}

static int can_speed(int index) {
  int ret;

  /*
     S0 = 10 kBaud
     S1 = 20 kBaud
     S2 = 50 kBaud
     S3 = 100 kBaud
     S4 = 125 kBaud
     S5 = 250 kBaud
     S6 = 500 kBaud
     S7 = 800 kBaud
     S8 = 1 MBaud

      //// Bit timing settings
      //// Assuming 48MHz base clock, 87.5% sample point, 500 kBit/s data rate
      //// http://www.bittiming.can-wiki.info/
      // Resync time quanta jump width
      // TODO - setup times not a factor of 2 from 500
TTCM: Time triggered comm mode
ABOM: Automatic bus-off management
AWUM: Automatic wakeup mode
NART: No automatic retransmission
RFLM: Receive FIFO locked mode
TXFP: Transmit FIFO priority
*/
  switch(index) {
    case 0: ret = can_init(CAN1, false, true, false, false, false, false, // TODO
                CAN_BTR_SJW_1TQ, CAN_BTR_TS1_14TQ, CAN_BTR_TS2_5TQ, 180, false, false);
            break;
    case 1: ret = can_init(CAN1, false, true, false, false, false, false, // TODO
                CAN_BTR_SJW_1TQ, CAN_BTR_TS1_14TQ, CAN_BTR_TS2_5TQ,  90, false, false);
            break;
    case 2: ret = can_init(CAN1, false, true, false, false, false, false,
                CAN_BTR_SJW_1TQ, CAN_BTR_TS1_11TQ, CAN_BTR_TS2_4TQ,  60, false, false);
            break;
    case 3: ret = can_init(CAN1, false, true, false, false, false, false, // TODO
                CAN_BTR_SJW_1TQ, CAN_BTR_TS1_8TQ, CAN_BTR_TS2_2TQ,   36, false, false);
            break;
    case 4: ret = can_init(CAN1, false, true, false, false, false, false,
                CAN_BTR_SJW_1TQ, CAN_BTR_TS1_11TQ, CAN_BTR_TS2_4TQ,  24, false, false);
            break;
    case 5: ret = can_init(CAN1, false, true, false, false, false, false,
                CAN_BTR_SJW_1TQ, // 16 number of time quanta
                CAN_BTR_TS1_11TQ, // 13
                CAN_BTR_TS2_4TQ, // 2
                12, false, false);
            break;
    case 6: ret = can_init(CAN1, false, true, false, false, false, false,
                CAN_BTR_SJW_1TQ, // 16 number of time quanta
                CAN_BTR_TS1_11TQ, // 13
                CAN_BTR_TS2_4TQ, // 2
                6, false, false);
            break;
    case 7: ret = can_init(CAN1, false, true, false, false, false, false, // TODO
                CAN_BTR_SJW_1TQ, CAN_BTR_TS1_12TQ, CAN_BTR_TS2_2TQ,   3, false, false);
            break;
    case 8: ret = can_init(CAN1, false, true, false, false, false, false,
                CAN_BTR_SJW_1TQ, CAN_BTR_TS1_11TQ, CAN_BTR_TS2_4TQ,   3, false, false);
            break;
    default:
            ret = -1;
            break;
  }
  return ret;
}

static void can_setup(void) {
  // enable CAN peripheral clock
  rcc_periph_clock_enable(RCC_CAN1);

  // Route the can to the relevant pins
  const uint16_t pins = GPIO8 | GPIO9;
  gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, pins);
  gpio_set_af(GPIOB, GPIO_AF4, pins);

  /* Reset CAN */
  can_reset(CAN1);

  /* defaultt CAN setting 250 kBaud */ 
  if (can_speed(5)) {
    gpio_set(GPIOB, GPIO1);	/* LED orange on */

    /* Die because we failed to initialize. */
    while (1)
      __asm__("nop");
  }

  /* CAN filter 0 init. */
  can_filter_id_mask_32bit_init(
      0,	/* Filter ID */
      0,	/* CAN ID */
      0,	/* CAN ID mask */
      0,	/* FIFO assignment (here: FIFO0) */
      true);	/* Enable the filter. */

  /* Enable CAN RX interrupt. */
  can_enable_irq(CAN1, CAN_IER_FMPIE0);
  // NVIC setup
  nvic_enable_irq(NVIC_CEC_CAN_IRQ);
  nvic_set_priority(NVIC_CEC_CAN_IRQ, 1);

}

void sys_tick_handler(void) {

  /* We call this handler every 1ms so every 1ms = 0.001s
   * resulting in 1Hz message rate.
   */

  counter++;
  if (!connected) {
    if (counter == 500) {
      counter = 0;
      gpio_toggle(GPIOB, GPIO0);	/* toggle green LED */
    }
  } else {
    // turn on when connected
    gpio_set(GPIOB, GPIO0);
  }
}

#if 0
static void gpio_debug(int n) {

  switch(n) {
    case 0:
      gpio_clear(GPIOC, GPIO14);
      gpio_clear(GPIOC, GPIO15);
      break;
    case 1:
      gpio_set(GPIOC, GPIO14);
      gpio_clear(GPIOC, GPIO15);
      break;
    case 2:
      gpio_clear(GPIOC, GPIO14);
      gpio_set(GPIOC, GPIO15);
      break;
    case -1:
      gpio_set(GPIOC, GPIO14);
      gpio_set(GPIOC, GPIO15);
      break;
  }
}
#endif

static void put_hex(uint8_t c) {
  uint8_t s[2];

  bin2hex(s, c);
  ring_write(&output_ring, s, 2);
}

void cec_can_isr(void) {
  uint32_t id;
  bool ext, rtr;
  uint8_t i, dlc, fmi, data[8];
  char c;
  uint16_t timestamp;

  can_receive(CAN1, 0, false, &id, &ext, &rtr, &fmi, &dlc, data, &timestamp);

  gpio_toggle(GPIOB, GPIO1);

  if (rtr) {
    if (ext)
      c = 'R';
    else
      c = 'r';
  } else {
    if (ext)
      c = 'T';
    else
      c = 't';
  }
  ring_write_ch(&output_ring, c);
  if (ext) {
    c = (id >> 24) & 0xff;
    put_hex(c);
    c = (id >> 16) & 0xff;
    put_hex(c);
    c = (id >> 8) & 0xff;
    put_hex(c);
    c = id & 0xff;
    put_hex(c);
  } else {
    /* bits 11-9 */
    c = (id >> 8) & 0x07;
    c += 0x30;
    ring_write_ch(&output_ring, c);
    /* bits 8-1 */
    c = id & 0xff;
    put_hex(c);
  }
  c = (dlc & 0x0f) | 0x30;
  ring_write_ch(&output_ring, c);
  for (i = 0 ; i < dlc; i++)
    put_hex(data[i]);

  ring_write_ch(&output_ring, '\r');

  can_fifo_release(CAN1, 0);

  /* enable the transmitter now */
  USART_CR1(USART2) |= USART_CR1_TXEIE;
}

static uint32_t get_nibbles(int nibbles) {
  int i;
  uint32_t id;
  char c;

  id = 0;
  for (i = 0; i < nibbles; i++) {
    c = ring_read_ch(&input_ring, NULL);
    id <<= 4;
    id |= nibble2bin(c);
  }
  return id;
}

static int slcan_command(void) {
  bool ext, rtr;
  uint8_t i, dlc, data[8];
  uint32_t id;
  int32_t ret;
  char c;
  bool send;

  connected = true;

  id = 0;
  dlc = 0;
  ext = true;
  send = true;
  rtr = false;

  if (!can_available_mailbox(CAN1))
    return -1;

  c = ring_read_ch(&input_ring, NULL);
  switch(c) {
    case 'T':
      id = get_nibbles(8);
      dlc = get_nibbles(1);
      break;
    case 't':
      ext = false;
      id = get_nibbles(3);
      dlc = get_nibbles(1);
      break;
    case 'R':
      rtr = true;
      ext = true;
      id = get_nibbles(8);
      dlc = get_nibbles(1);
      break;
    case 'r':
      rtr = true;
      ext = false;
      id = get_nibbles(3);
      dlc = get_nibbles(1);
      break;
    case 'S':
      c = get_nibbles(1);
      can_speed(c);
      send = false;
      break;
    case 'v':
      send = false;
      break;
    case 'V':
      send = false;
      break;
    case 'C':
      send = false;
      break;
    default:
      send = false;
      break;
  }

  for ( i = 0; i < dlc; i++) {
    data[i] = (uint8_t) get_nibbles(2); 
  }

  /* consume chars until eol reached */
  do {
    ret = ring_read_ch(&input_ring, NULL);
  } while (ret == '\r');

#if 1
  if (send) {
    ret = can_transmit(CAN1, id, ext, rtr, dlc, data);
    /* gpio_debug(ret); */
  }
#else
  if (send) {
    int loop = CAN_MAX_RETRY;
    /* try to send data - omit if not possible */
    while(loop-- > 0) {
      if (can_available_mailbox(CAN1))
        break;
      /* TODO: LED overflow */
    }
    ret = can_transmit(CAN1, id, ext, rtr, dlc, data);
    gpio_debug(ret);
  }
#endif

  if (commands_pending)
    commands_pending--;

  return 0;
}

int main(void) {
  status = 0;
  commands_pending = 0;

  rcc_clock_setup_in_hsi_out_48mhz();
  gpio_setup();
  can_setup();
  usart_setup();

  // just for blinky led
  systick_setup();

  /* endless loop */
  while (1) {
    if (commands_pending)
      slcan_command();
  }
  return 0;
}
