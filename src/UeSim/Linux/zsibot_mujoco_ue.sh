#!/bin/bash
set -euo pipefail

UE_TRUE_SCRIPT_NAME="$(readlink -f "$0")"
UE_PROJECT_ROOT="$(dirname "$UE_TRUE_SCRIPT_NAME")"
UE_BIN_DIR="$UE_PROJECT_ROOT/zsibot_mujoco_ue/Binaries/Linux"

# 不同打包配置下，UE 可执行文件的命名不完全一致。
# 这里按常见名字依次尝试，避免 launcher 只认 Shipping 产物。
for candidate in \
    "$UE_BIN_DIR/zsibot_mujoco_ue-Linux-Shipping" \
    "$UE_BIN_DIR/zsibot_mujoco_ue-Linux-Development" \
    "$UE_BIN_DIR/zsibot_mujoco_ue"
do
    if [ -f "$candidate" ]; then
        chmod +x "$candidate"

        # 显式设置 LD_LIBRARY_PATH，包含常用的库路径，确保在不同环境下都能找到依赖
        export LD_LIBRARY_PATH="$UE_BIN_DIR:$UE_PROJECT_ROOT/Engine/Binaries/Linux:$UE_PROJECT_ROOT/Engine/Plugins/Runtime/OpenCV/Binaries/ThirdParty/Linux:${LD_LIBRARY_PATH:-}"

        exec "$candidate" zsibot_mujoco_ue "$@"
    fi
done

echo "ERROR: 找不到 UE 可执行文件，已尝试:"
echo "  - $UE_BIN_DIR/zsibot_mujoco_ue-Linux-Shipping"
echo "  - $UE_BIN_DIR/zsibot_mujoco_ue-Linux-Development"
echo "  - $UE_BIN_DIR/zsibot_mujoco_ue"
exit 1
