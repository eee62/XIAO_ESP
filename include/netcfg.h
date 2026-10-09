// The node's home-network address, settable from deployment mode.
//
// PROJECT_BRIEF.md 9.4 wants a static address on the fast path; config.h's
// NET_STATIC_IP block is the built-in one. This lets the operator replace it
// from the CAM-SETUP page without a reflash: the values are kept in NVS
// (namespace "net"), which survives deep sleep, resets, a dead battery and a
// normal upload, and are used in place of config.h's until they are forgotten.
//
// Addresses are IPv4 as a.b.c.d packed (a << 24) | (b << 16) | (c << 8) | d.
// The parsing and checking below are plain C++ (src/netcfg.cpp), so
// tools/netcfg_host_test.sh can exercise them; the NVS side is
// src/netcfg_store.cpp.
#pragma once

#include <stdint.h>

struct netcfg_t {
	uint32_t ip;
	uint32_t gw;
	uint32_t sn;
	uint32_t dns;
};

// "192.168.1.50" -> 0xC0A80132. Exactly four decimal numbers 0-255, dots
// between, nothing else (no spaces, signs, leading '+', or a fifth part).
bool netcfg_parse_ip(const char *s, uint32_t *out);
// The reverse, into at least 16 bytes.
void netcfg_format(uint32_t a, char *buf);

// The defaults for fields left empty: gateway = the IP's first three numbers
// with .1, DNS = the gateway, subnet = 255.255.255.0.
uint32_t netcfg_default_gw(uint32_t ip);
#define NETCFG_DEFAULT_SN 0xFFFFFF00u

// nullptr when `c` is usable as a static configuration, else a one-line
// reason fit for the page's toast. The rules: no 0.0.0.0, loopback or
// multicast address; a contiguous subnet mask leaving at least two host bits;
// the IP and the gateway each neither the network nor the broadcast address
// of that subnet; the gateway inside the IP's subnet; IP and gateway differ.
const char *netcfg_check(const netcfg_t &c);

// config.h's NET_* block.
void netcfg_builtin(netcfg_t *out);

// The saved configuration. False when none is saved, or when what is there
// fails netcfg_check() (then logged, and config.h's applies). Reads only.
bool netcfg_load(netcfg_t *out);
// Save `c` (already checked). Writes only the keys whose value differs, and
// the "set" flag only if it is not already set; *changed says whether anything
// was written. False on an NVS error.
bool netcfg_save(const netcfg_t &c, bool *changed);
// Forget the saved configuration (erase namespace "net"). *had says whether
// there was one. False on an NVS error.
bool netcfg_clear(bool *had);
