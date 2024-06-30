#include "spin_coater.h"
#include "stdio.h"

#include "spin_coater_dshot.h"

#include "hardware/pio.h"

#define MICROSECONDS_PER_MINUTE 60000000
static PIO pio = pio0;
static int pio_sm = -1;

#if  CONFIG_SPIN_COATER == CONFIG_SPIN_COATER_DSHOT

bool
dshot_init(uint16_t dshot_gpio, uint16_t dshot_gpio_reversed)
{
  uint pio_offset = pio_add_program(pio, &dshot_encoder_program);
  pio_sm = pio_claim_unused_sm(pio, true);

  if (pio_sm < 0) {
    pio_sm_unclaim(pio, pio_sm);
    return false;
  }

  dshot_encoder_program_init(pio, pio_sm, pio_offset, dshot_gpio);
  return true;
}

void
dshot_send_command(uint16_t c)
{
  // Shift for telemetry bit (0)
  c = c << 1;

  // Shift and include checksum
  uint16_t checksum = (c ^ (c >> 4) ^ (c >> 8)) & 0x0F;
  c = (c << 4) | checksum;

  pio_sm_put_blocking(pio, pio_sm, c);
}
#elif  CONFIG_SPIN_COATER == CONFIG_SPIN_COATER_BIDIR_DSHOT

bool
dshot_init(uint16_t dshot_gpio, uint16_t dshot_gpio_reversed)
{
  uint pio_offset = pio_add_program(pio, &dshot_encoder_program);
  pio_sm = pio_claim_unused_sm(pio, true);

  if (pio_sm < 0) {
    pio_sm_unclaim(pio, pio_sm);
    return false;
  }

  bidir_dshot_encoder_program_init(pio, pio_sm, pio_offset, dshot_gpio, dshot_gpio_reversed);
  return true;
}

void
dshot_send_command(uint16_t c)
{
  // Shift for telemetry bit (0)
  c = c << 1;

  // Shift and include checksum
  uint16_t checksum = (~(c ^ (c >> 4) ^ (c >> 8))) & 0x0F;
  c = (c << 4) | checksum;

  pio_sm_put_blocking(pio, pio_sm, c);
}

#define GET_GCR_CHUNK(GCR, CHUNK_NUM) (GCR >> (5*CHUNK_NUM) & 0x1f)
static
uint8_t calc_crc(uint16_t value)
{
  return (~(value ^ (value >> 4) ^ (value >> 8))) & 0x0F;
}

static const uint32_t decodeGCR[32] = {
    0, 0, 0, 0, 0, 0, 0, 0, 0, 9, 10, 11, 0, 13, 14, 15,
    0, 0, 2, 3, 0, 5, 6, 7, 0, 0, 8, 1, 0, 4, 12, 0 };

static uint32_t dshot_decode_eRPM_telemetry_value(uint32_t value)
{
  // first decode custom encoding from 21 bit value to 20 bit gcr code
  uint32_t gcr = (value ^ (value >> 1));

  // returned telemetry 4x4bits is encoded as 20bit gcr encoded value 4 x 5bits
  uint32_t raw_telemetry = decodeGCR[gcr & 0x1f];
  raw_telemetry |= decodeGCR[(gcr >> 5) & 0x1f] << 4;
  raw_telemetry |= decodeGCR[(gcr >> 10) & 0x1f] << 8;
  raw_telemetry |= decodeGCR[(gcr >> 15) & 0x1f] << 12;

  uint16_t obtained_crc = raw_telemetry & 0xf;
  uint16_t encoded_erpm = raw_telemetry >> 4;

  if(calc_crc(encoded_erpm) != obtained_crc)
    return 0;

  // Currecntly handle only eRPM packets
  if(encoded_erpm  == 0x0fff)
    return 0;

  uint32_t erpm_decoded = (encoded_erpm & 0x000001ff) << ((encoded_erpm & 0xfffffe00) >> 9);
  const int number_of_poles = 14;

  return MICROSECONDS_PER_MINUTE/(erpm_decoded*number_of_poles/2.0);
}

// Currectly only erpm is supported
uint32_t
dshot_recv_telemetry()
{
  // in fifo there is up to 4 entries
  // so at most there can be all 4 old values
  // throw old values and read new one
  for(int i =0 ; i < 4; i++)
    printf("Ignoring: %u\n", pio_sm_get_blocking(pio, pio_sm));

  uint32_t val = pio_sm_get_blocking(pio, pio_sm);
  printf("RAW:%u - ", val);
  return dshot_decode_eRPM_telemetry_value(val);
}

#endif