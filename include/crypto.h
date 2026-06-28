#pragma once
#include <stdint.h>
#include <string.h>

// ============================================================================
//  ChaCha20 keystream (RFC 8439 core) — used to encrypt the audio payload.
//
//  This is a standard, well-reviewed stream cipher. We use it in keystream/CTR
//  fashion: each packet carries a 32-bit nonce (a monotonic counter, in clear)
//  that, together with the channel index and the pre-shared key, deterministically
//  selects a 64-byte keystream block. The transmitter and receiver derive the
//  same block from clear header fields, so every packet is independently
//  decryptable — a lost or out-of-order packet never desyncs the cipher.
//
//  Threat model: this protects intelligible audio from a casual listener with
//  another nRF24 on the same channel/address. It is NOT authenticated encryption
//  (the 8-bit app CRC gives ~1/256 wrong-key/tamper detection, not real MAC
//  security) and the nonce space is 32-bit, so it is "privacy", not milspec.
// ============================================================================

static inline uint32_t cc_rotl(uint32_t x, int n) { return (x << n) | (x >> (32 - n)); }

#define CC_QR(a,b,c,d) \
    a += b; d ^= a; d = cc_rotl(d,16); \
    c += d; b ^= c; b = cc_rotl(b,12); \
    a += b; d ^= a; d = cc_rotl(d, 8); \
    c += d; b ^= c; b = cc_rotl(b, 7);

// Produce one 64-byte keystream block.
//   key   : 32 bytes (pre-shared)
//   counter: 32-bit block counter (we use 0 — one block is plenty for <=64 bytes)
//   nonce : 12 bytes (we pack the 32-bit packet nonce + channel here)
static inline void chacha20_block(const uint8_t key[32], uint32_t counter,
                                  const uint8_t nonce[12], uint8_t out[64]) {
    static const uint32_t c[4] = {0x61707865, 0x3320646e, 0x79622d32, 0x6b206574};
    uint32_t s[16];
    s[0]=c[0]; s[1]=c[1]; s[2]=c[2]; s[3]=c[3];
    for (int i = 0; i < 8; i++)
        s[4+i] = (uint32_t)key[i*4] | ((uint32_t)key[i*4+1]<<8) |
                 ((uint32_t)key[i*4+2]<<16) | ((uint32_t)key[i*4+3]<<24);
    s[12] = counter;
    for (int i = 0; i < 3; i++)
        s[13+i] = (uint32_t)nonce[i*4] | ((uint32_t)nonce[i*4+1]<<8) |
                  ((uint32_t)nonce[i*4+2]<<16) | ((uint32_t)nonce[i*4+3]<<24);

    uint32_t w[16];
    memcpy(w, s, sizeof(w));
    for (int i = 0; i < 10; i++) {       // 20 rounds = 10 double-rounds
        CC_QR(w[0], w[4], w[ 8], w[12]);
        CC_QR(w[1], w[5], w[ 9], w[13]);
        CC_QR(w[2], w[6], w[10], w[14]);
        CC_QR(w[3], w[7], w[11], w[15]);
        CC_QR(w[0], w[5], w[10], w[15]);
        CC_QR(w[1], w[6], w[11], w[12]);
        CC_QR(w[2], w[7], w[ 8], w[13]);
        CC_QR(w[3], w[4], w[ 9], w[14]);
    }
    for (int i = 0; i < 16; i++) {
        uint32_t v = w[i] + s[i];
        out[i*4]   = (uint8_t)(v);
        out[i*4+1] = (uint8_t)(v >> 8);
        out[i*4+2] = (uint8_t)(v >> 16);
        out[i*4+3] = (uint8_t)(v >> 24);
    }
}

// XOR a small (<=64 byte) buffer in place with the keystream selected by
// (key, packetNonce, channel). Symmetric: same call encrypts and decrypts.
static inline void crypto_xor(const uint8_t key[32], uint32_t packetNonce,
                              uint8_t channel, uint8_t *data, uint8_t len) {
    if (len == 0 || len > 64) return;
    uint8_t nonce[12] = {0};
    nonce[0] = (uint8_t)(packetNonce);
    nonce[1] = (uint8_t)(packetNonce >> 8);
    nonce[2] = (uint8_t)(packetNonce >> 16);
    nonce[3] = (uint8_t)(packetNonce >> 24);
    nonce[4] = channel;                 // domain-separate channels
    uint8_t ks[64];
    chacha20_block(key, 0, nonce, ks);
    for (uint8_t i = 0; i < len; i++) data[i] ^= ks[i];
}

#undef CC_QR
