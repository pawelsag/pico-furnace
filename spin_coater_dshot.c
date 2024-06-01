#include "spin_coater.h"

#include "spin_coater_dshot.h"

#include "hardware/pio.h"

#define MICROSECONDS_PER_MINUTE 60000000
static PIO pio = pio0;
static int pio_sm = -1;

#if  CONFIG_SPIN_COATER == CONFIG_SPIN_COATER_DSHOT

bool
dshot_init(uint16_t dshot_gpio)
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
dshot_init(uint16_t dshot_gpio)
{
  uint pio_offset = pio_add_program(pio, &dshot_encoder_program);
  pio_sm = pio_claim_unused_sm(pio, true);

  if (pio_sm < 0) {
    pio_sm_unclaim(pio, pio_sm);
    return false;
  }

  bidir_dshot_encoder_program_init(pio, pio_sm, pio_offset, dshot_gpio);
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

uint8_t decode_GCR_nibble(uint8_t gcr) {
  switch(gcr)	{
    case 0x19:
      return 0x0;
    case 0x1B:
      return 0x1;
    case 0x12:
      return 0x2;
    case 0x13:
      return 0x3;
    case 0x1D:
      return 0x4;
    case 0x15:
      return 0x5;
    case 0x16:
      return 0x6;
    case 0x17:
      return 0x7;
    case 0x1A:
      return 0x8;
    case 0x09:
      return 0x9;
    case 0x0A:
      return 0xA;
    case 0x0B:
      return 0xB;
    case 0x1E:
      return 0xC;
    case 0x0D:
      return 0xD;
    case 0x0E:
      return 0xE;
    case 0x0F:
      return 0xF;

    default:
      return 0xFF;
  }
}

#define GET_GCR_CHUNK(GCR, CHUNK_NUM) (GCR >> (5*CHUNK_NUM) & 0x1f)

uint8_t calc_crc(uint16_t value)
{
  return (~(value ^ (value >> 4) ^ (value >> 8))) & 0x0F;
}

static uint32_t dshot_decode_eRPM_telemetry_value(uint32_t value)
{
  // first decode custom encoding from 21 bit value to 20 bit gcr code
  uint32_t gcr = (value ^ (value >> 1));

  uint16_t raw_telemetry = 0x0;
  // returned telemetry 4x4bits is encoded as 20bit gcr encoded value 4 x 5bits
  for(int i =0 ; i < 4; i++)
    raw_telemetry |= (decode_GCR_nibble(GET_GCR_CHUNK(gcr, i)) << 4*i);
  uint16_t obtained_crc = raw_telemetry & 0xff;
  uint16_t encoded_erpm = raw_telemetry >> 4;

  if(calc_crc(encoded_erpm) != obtained_crc)
    return 0;

  // Currecntly handle only eRPM packets
  if((encoded_erpm & 0x100) == 0) 
    return 0;

  uint8_t exponent = (encoded_erpm >> 7);
  uint8_t base = (encoded_erpm & 0x1ff);
  uint32_t erpm_decoded = base << exponent;
  const int number_of_poles = 3;

  return MICROSECONDS_PER_MINUTE/(erpm_decoded*number_of_poles/2);
}

// Currectly only erpm is supported
uint32_t
dshot_recv_telemetry()
{
  // in fifo there is up to 4 entries
  // so at most there can be all 4 old values
  // throw old values and read new one
  for(int i =0 ; i < 4; i++)
    pio_sm_get_blocking(pio, pio_sm);

  return dshot_decode_eRPM_telemetry_value(pio_sm_get_blocking(pio, pio_sm));
}

#endif