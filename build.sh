#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
sdk_root="${DOG_SDK_ROOT:-$HOME/Projects/dg_fsm_legged}"
include_root="$sdk_root/robot/dds_comm"
lib_dir="$include_root/thirdparty/lib/x86_64"

g++ -std=c++17 -O2 -Wall -Wextra -pthread \
  -I"$include_root/include" \
  -I"$include_root/ltr_m1/include" \
  -isystem "$include_root/thirdparty/include" \
  -isystem "$include_root/thirdparty/include/ddscxx" \
  "$script_dir/dds_bridge.cpp" \
  "$include_root/lib/x86_64/libltr_sdk.a" \
  -L"$lib_dir" -Wl,--disable-new-dtags,-rpath,"$lib_dir" \
  -lddscxx -lddsc -pthread \
  -o "$script_dir/dds_bridge"

echo "Built $script_dir/dds_bridge"
