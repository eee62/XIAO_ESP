// The home-network address: NVS storage (netcfg.h), with the Preferences
// library the Arduino core ships. NVS itself is initialised by the core on
// every boot (initArduino()), before setup(), so a read here costs only the
// namespace open and four key lookups.

#include <Arduino.h>
#include <Preferences.h>

#include "config.h"
#include "netcfg.h"

#define NETCFG_NS "net"

static uint32_t pack(uint8_t a, uint8_t b, uint8_t c, uint8_t d)
{
	return ((uint32_t)a << 24) | ((uint32_t)b << 16) | ((uint32_t)c << 8) | d;
}

// config.h spells each address as four comma-separated numbers.
static uint32_t pack4(const uint8_t (&v)[4])
{
	return pack(v[0], v[1], v[2], v[3]);
}

void netcfg_builtin(netcfg_t *out)
{
	static const uint8_t ip[4] = {NET_STATIC_IP}, gw[4] = {NET_GATEWAY},
	                     sn[4] = {NET_SUBNET}, dns[4] = {NET_DNS};
	out->ip  = pack4(ip);
	out->gw  = pack4(gw);
	out->sn  = pack4(sn);
	out->dns = pack4(dns);
}

bool netcfg_load(netcfg_t *out)
{
	Preferences p;
	// Read-only: never creates the namespace. On a node that has never had
	// an address saved the open fails, and Preferences logs that once
	// ("nvs_open failed: NOT_FOUND"); it means "nothing saved", nothing worse.
	if (!p.begin(NETCFG_NS, true)) {
		return false;
	}
	const bool set = p.getBool("set", false);
	netcfg_t c;
	c.ip  = p.getUInt("ip", 0);
	c.gw  = p.getUInt("gw", 0);
	c.sn  = p.getUInt("sn", 0);
	c.dns = p.getUInt("dns", 0);
	p.end();
	if (!set) {
		return false;
	}
	const char *why = netcfg_check(c);
	if (why) {
		log_w("net: saved address ignored (%s); using config.h's", why);
		return false;
	}
	*out = c;
	return true;
}

bool netcfg_save(const netcfg_t &c, bool *changed)
{
	*changed = false;
	Preferences p;
	if (!p.begin(NETCFG_NS, false)) {
		return false;
	}
	// Flash wear: a key is written only when its value is new.
	const struct {
		const char *key;
		uint32_t    val;
	} keys[] = {{"ip", c.ip}, {"gw", c.gw}, {"sn", c.sn}, {"dns", c.dns}};
	bool ok = true;
	for (const auto &k : keys) {
		if (!p.isKey(k.key) || p.getUInt(k.key, 0) != k.val) {
			ok &= p.putUInt(k.key, k.val) == sizeof(uint32_t);
			*changed = true;
		}
	}
	if (!p.getBool("set", false)) {
		ok &= p.putBool("set", true) == 1;
		*changed = true;
	}
	p.end();
	return ok;
}

bool netcfg_clear(bool *had)
{
	Preferences p;
	*had = false;
	// Read-only first: forgetting with nothing saved must not create the
	// namespace (a flash write) just to empty it.
	if (!p.begin(NETCFG_NS, true)) {
		return true;
	}
	*had = p.getBool("set", false) || p.isKey("ip");
	p.end();
	if (!*had) {
		return true;
	}
	if (!p.begin(NETCFG_NS, false)) {
		return false;
	}
	const bool ok = p.clear();
	p.end();
	return ok;
}
