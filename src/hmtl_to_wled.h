#pragma once
//
// hmtl_to_wled.h — parse an HMTL OUTPUT message into a colour the WLED side can use.
// Pure, host-testable, and deliberately byte-level: this reads the wire directly rather than
// casting to the firmware structs, so it cannot be broken by a packing or ABI difference between
// the coordinator's build and the AVR modules'.
//
// Wire layout, read from HMTL/Libraries/HMTLProtocol/HMTLWireFormat.h:
//
//   msg_hdr_t (8 bytes, packed)          output_hdr_t (2 bytes)     msg_rgb_t adds
//     0  startcode  0xFC (:194)            0  type                     0..2  r, g, b
//     1  crc                               1  output
//     2  version    HMTL_MSG_VERSION 2
//     3  length
//     4  type       MSG_TYPE_OUTPUT 0x01 (:214)
//     5  flags
//     6  address    uint16 LE
//
//   HMTL_OUTPUT_RGB 0x2 (:150)   HMTL_ALL_OUTPUTS 0xFE (:169)   HMTL_NO_OUTPUT 0xFF (:168)
//
#include <stdint.h>
#include <stddef.h>

#define HMTL_WIRE_START        0xFCu
#define HMTL_WIRE_VERSION      2u
#define HMTL_WIRE_MSG_OUTPUT   0x01u
#define HMTL_WIRE_OUTPUT_RGB   0x02u
#define HMTL_WIRE_OUTPUT_VALUE 0x01u
#define HMTL_WIRE_ALL_OUTPUTS  0xFEu
#define HMTL_WIRE_NO_OUTPUT    0xFFu

#define HMTL_HDR_LEN       8u
#define HMTL_OUTPUT_HDR_LEN 2u
#define HMTL_RGB_MSG_LEN   (HMTL_HDR_LEN + HMTL_OUTPUT_HDR_LEN + 3u)   /* 13 */

enum hmtl_parse_result_t {
  HMTL_PARSE_OK = 0,
  HMTL_PARSE_TOO_SHORT,      // fewer bytes than the layout requires
  HMTL_PARSE_BAD_START,
  HMTL_PARSE_BAD_VERSION,
  HMTL_PARSE_NOT_OUTPUT,     // a valid HMTL message, but not an OUTPUT
  HMTL_PARSE_NOT_RGB,        // an OUTPUT, but not one that carries a colour
  HMTL_PARSE_LENGTH_MISMATCH // the header's own length disagrees with the buffer
};

struct hmtl_rgb_msg_t {
  uint16_t address;
  uint8_t  output;    // HMTL_WIRE_ALL_OUTPUTS for a broadcast
  uint8_t  r, g, b;
};

// Parse `buf` as an HMTL RGB OUTPUT message.
//
// Every field is bounds-checked BEFORE it is read; `len` is never trusted from the header alone.
// A malformed frame is a reported reason rather than a silent zero-colour, because "everything went
// black" and "I ignored your message" look identical on a light otherwise.
static inline hmtl_parse_result_t hmtl_parse_rgb(const uint8_t *buf, size_t len,
                                                 hmtl_rgb_msg_t *out) {
  if (buf == 0 || out == 0) return HMTL_PARSE_TOO_SHORT;
  if (len < HMTL_HDR_LEN) return HMTL_PARSE_TOO_SHORT;

  if (buf[0] != HMTL_WIRE_START)   return HMTL_PARSE_BAD_START;
  if (buf[2] != HMTL_WIRE_VERSION) return HMTL_PARSE_BAD_VERSION;
  if (buf[4] != HMTL_WIRE_MSG_OUTPUT) return HMTL_PARSE_NOT_OUTPUT;

  /* The header carries its own length. Trusting the buffer size alone would accept a frame that
   * claims to be shorter than it is and read a stale tail as colour. */
  const uint8_t claimed = buf[3];
  if (claimed != len) return HMTL_PARSE_LENGTH_MISMATCH;

  if (len < HMTL_RGB_MSG_LEN) return HMTL_PARSE_TOO_SHORT;

  const uint8_t out_type = buf[HMTL_HDR_LEN + 0];
  if (out_type != HMTL_WIRE_OUTPUT_RGB) return HMTL_PARSE_NOT_RGB;

  out->address = (uint16_t)((uint16_t)buf[6] | ((uint16_t)buf[7] << 8));  /* little-endian */
  out->output  = buf[HMTL_HDR_LEN + 1];
  out->r       = buf[HMTL_HDR_LEN + 2];
  out->g       = buf[HMTL_HDR_LEN + 3];
  out->b       = buf[HMTL_HDR_LEN + 4];
  return HMTL_PARSE_OK;
}

// Whether a parsed message should drive every WLED device this coordinator knows about.
//
// The coordinator has no outputs of its own, so a message aimed at a SPECIFIC output number is not
// addressed to anything it owns and is not a fleet colour change. Only the broadcast sentinel is
// treated as "set everything"; anything else is deliberately ignored rather than guessed at, since
// guessing would turn one module's cue into a whole-installation colour change.
static inline bool hmtl_rgb_is_fleet_wide(const hmtl_rgb_msg_t *m) {
  return m != 0 && m->output == HMTL_WIRE_ALL_OUTPUTS;
}
