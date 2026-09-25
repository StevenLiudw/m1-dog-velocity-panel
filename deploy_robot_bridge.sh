#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
target="${DOG_ROBOT_SSH_TARGET:?Set DOG_ROBOT_SSH_TARGET to USER@ROBOT_IP}"

scp -o StrictHostKeyChecking=yes "$script_dir/dds_bridge.cpp" "$target:/tmp/dog_dds_bridge.cpp"
ssh -o StrictHostKeyChecking=yes "$target" 'bash -s' <<'REMOTE'
set -euo pipefail
sdk="$HOME/dg_fsm_m1/robot/dds_comm"
g++ -std=c++17 -O2 -pthread \
  -I"$sdk/include" -I"$sdk/ltr_m1/include" \
  -isystem "$sdk/thirdparty/include" \
  -isystem "$sdk/thirdparty/include/ddscxx" \
  /tmp/dog_dds_bridge.cpp "$sdk/lib/aarch64/libltr_sdk.a" \
  -L"$sdk/thirdparty/lib/aarch64" \
  -Wl,--disable-new-dtags,-rpath,"$sdk/thirdparty/lib/aarch64" \
  -lddscxx -lddsc -o /tmp/dog_dds_bridge
install -m 755 /tmp/dog_dds_bridge "$HOME/dg_fsm_m1/build/test_repo/dog_dds_bridge.new"
mv "$HOME/dg_fsm_m1/build/test_repo/dog_dds_bridge.new" "$HOME/dg_fsm_m1/build/test_repo/dog_dds_bridge"
REMOTE

echo "Installed auxiliary DDS publisher on $target"
