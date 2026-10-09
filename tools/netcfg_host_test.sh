#!/bin/sh
# Host test for the home-network address checks (src/netcfg.cpp): builds them
# with the host compiler and runs tools/netcfg_host_test.cpp.
#
#   tools/netcfg_host_test.sh
set -eu

root=$(cd "$(dirname "$0")/.." && pwd)
work=$(mktemp -d)
trap 'rm -rf "$work"' EXIT

c++ -std=c++17 -O1 -Wall -Wextra -Werror -fsanitize=address,undefined \
	-I"$root/include" \
	"$root/src/netcfg.cpp" "$root/tools/netcfg_host_test.cpp" \
	-o "$work/netcfg_host_test"
"$work/netcfg_host_test"
