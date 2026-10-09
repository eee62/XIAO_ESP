// The home-network address: parsing and checking (netcfg.h). No Arduino or
// ESP-IDF here, so tools/netcfg_host_test.sh can build it on the host.

#include "netcfg.h"

#include <stdio.h>

bool netcfg_parse_ip(const char *s, uint32_t *out)
{
	if (!s) {
		return false;
	}
	uint32_t a = 0;
	for (int part = 0; part < 4; part++) {
		if (part > 0) {
			if (*s != '.') {
				return false;
			}
			s++;
		}
		if (*s < '0' || *s > '9') {
			return false;
		}
		uint32_t v = 0;
		int digits = 0;
		while (*s >= '0' && *s <= '9') {
			if (++digits > 3) {
				return false;
			}
			v = v * 10 + (uint32_t)(*s++ - '0');
		}
		if (v > 255) {
			return false;
		}
		a = (a << 8) | v;
	}
	if (*s != '\0') {
		return false;
	}
	*out = a;
	return true;
}

void netcfg_format(uint32_t a, char *buf)
{
	snprintf(buf, 16, "%u.%u.%u.%u", (unsigned)(a >> 24), (unsigned)((a >> 16) & 255),
	         (unsigned)((a >> 8) & 255), (unsigned)(a & 255));
}

uint32_t netcfg_default_gw(uint32_t ip)
{
	return (ip & 0xFFFFFF00u) | 1u;
}

// 0.0.0.0/8, 127/8 and 224 and up (multicast, reserved, broadcast).
static bool unusable(uint32_t a)
{
	const uint32_t first = a >> 24;
	return first == 0 || first == 127 || first >= 224;
}

const char *netcfg_check(const netcfg_t &c)
{
	if (c.ip == 0) {
		return "IP address cannot be 0.0.0.0";
	}
	if (unusable(c.ip)) {
		return "IP address is not a home-network address (0.x, 127.x or 224 and up)";
	}
	// A mask is ones then zeros: its complement plus one is a power of two.
	// At least two host bits, or there is no host between network and
	// broadcast.
	const uint32_t host = ~c.sn;
	if (c.sn == 0 || (host & (host + 1)) != 0) {
		return "Subnet must be a mask like 255.255.255.0";
	}
	if (host < 3) {
		return "Subnet is too small: use 255.255.255.252 or wider";
	}
	if ((c.ip & host) == 0) {
		return "IP address is the network address of its subnet";
	}
	if ((c.ip & host) == host) {
		return "IP address is the broadcast address of its subnet";
	}
	if (c.gw == 0) {
		return "Gateway cannot be 0.0.0.0";
	}
	if ((c.gw & c.sn) != (c.ip & c.sn)) {
		return "Gateway is outside the IP address's subnet";
	}
	if ((c.gw & host) == 0 || (c.gw & host) == host) {
		return "Gateway is the network or broadcast address of the subnet";
	}
	if (c.gw == c.ip) {
		return "IP address must differ from the gateway";
	}
	if (c.dns == 0 || unusable(c.dns)) {
		return "DNS must be a usable address (often the gateway)";
	}
	return nullptr;
}
