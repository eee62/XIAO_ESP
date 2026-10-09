// Host test for the home-network address checks (src/netcfg.cpp). Built and
// run by tools/netcfg_host_test.sh: the inputs deployment mode must refuse,
// and the ones it must take.

#include <stdio.h>
#include <string.h>

#include "netcfg.h"

static int g_fail = 0;

static void check(bool ok, const char *what)
{
	printf("%s  %s\n", ok ? "ok  " : "FAIL", what);
	if (!ok) {
		g_fail++;
	}
}

static uint32_t ip(const char *s)
{
	uint32_t a = 0xDEADBEEF;
	netcfg_parse_ip(s, &a);
	return a;
}

// The page's rules: empty gateway, DNS and subnet take their defaults.
static const char *verdict(const char *i, const char *g = "", const char *d = "",
                           const char *m = "")
{
	netcfg_t c;
	if (!netcfg_parse_ip(i, &c.ip)) {
		return "parse";
	}
	c.gw  = *g ? ip(g) : netcfg_default_gw(c.ip);
	c.sn  = *m ? ip(m) : NETCFG_DEFAULT_SN;
	c.dns = *d ? ip(d) : c.gw;
	return netcfg_check(c);
}

int main()
{
	uint32_t a;
	check(netcfg_parse_ip("192.168.1.50", &a) && a == 0xC0A80132u, "parse 192.168.1.50");
	check(!netcfg_parse_ip("300.1.1.1", &a), "300.1.1.1 refused");
	check(!netcfg_parse_ip("192.168.1", &a), "three parts refused");
	check(!netcfg_parse_ip("192.168.1.1.1", &a), "five parts refused");
	check(!netcfg_parse_ip("192.168.1.x", &a), "letters refused");
	check(!netcfg_parse_ip(" 192.168.1.5", &a), "leading space refused");
	check(!netcfg_parse_ip("192.168.1.5 ", &a), "trailing space refused");
	check(!netcfg_parse_ip("192..1.5", &a), "empty part refused");
	check(!netcfg_parse_ip("-1.2.3.4", &a), "sign refused");
	check(!netcfg_parse_ip("0192.168.1.5", &a), "four digits refused");
	check(!netcfg_parse_ip("", &a), "empty refused");
	char buf[16];
	netcfg_format(0xC0A80132u, buf);
	check(strcmp(buf, "192.168.1.50") == 0, "format round trip");

	check(verdict("192.168.1.50") == nullptr, "192.168.1.50 with defaults: ok");
	check(verdict("10.0.0.20", "10.0.0.1", "1.1.1.1", "255.255.0.0") == nullptr,
	      "10.0.0.20 /16, external DNS: ok");
	check(verdict("0.0.0.0") != nullptr, "0.0.0.0 refused");
	check(verdict("192.168.1.1") != nullptr, "the default gateway itself refused");
	check(verdict("192.168.1.50", "192.168.1.50") != nullptr, "IP equal to gateway refused");
	check(verdict("192.168.2.50", "192.168.1.1") != nullptr, "IP outside the gateway's net refused");
	check(verdict("192.168.1.0", "192.168.1.1") != nullptr, "network address refused");
	check(verdict("192.168.1.255", "192.168.1.1") != nullptr, "broadcast address refused");
	check(verdict("192.168.1.50", "192.168.1.255") != nullptr, "broadcast gateway refused");
	check(verdict("192.168.1.50", "", "", "255.0.255.0") != nullptr, "non-contiguous mask refused");
	check(verdict("192.168.1.50", "", "", "255.255.255.255") != nullptr, "/32 mask refused");
	check(verdict("127.0.0.5", "127.0.0.1") != nullptr, "loopback refused");
	check(verdict("224.0.0.5", "224.0.0.1") != nullptr, "multicast refused");
	check(verdict("192.168.1.50", "192.168.1.1", "0.0.0.0") != nullptr, "DNS 0.0.0.0 refused");
	check(verdict("192.168.1.130", "192.168.1.129", "", "255.255.255.128") == nullptr,
	      "/25 upper half: ok");
	check(verdict("192.168.1.130", "", "", "255.255.255.128") != nullptr,
	      "/25 upper half with the default .1 gateway: outside, refused");

	printf("%s: %d failure(s)\n", g_fail ? "FAILED" : "passed", g_fail);
	return g_fail ? 1 : 0;
}
