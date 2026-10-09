#!/bin/sh
# Host test for the reply reader (src/tg_updates.cpp): builds it with the host
# compiler and runs tools/tg_updates_host_test.cpp, which feeds it fresh, stale,
# foreign and malformed getUpdates bodies. Nothing here touches Telegram.
#
#   tools/tg_updates_host_test.sh
set -eu

root=$(cd "$(dirname "$0")/.." && pwd)
work=$(mktemp -d)
trap 'rm -rf "$work"' EXIT

c++ -std=c++17 -O1 -Wall -Wextra -Werror -fsanitize=address,undefined \
	-I"$root/include" \
	"$root/src/tg_updates.cpp" "$root/tools/tg_updates_host_test.cpp" \
	-o "$work/tg_updates_host_test"
"$work/tg_updates_host_test"
