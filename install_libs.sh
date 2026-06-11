#!/bin/sh
#
# 把 tennis 运行所需的 5 个 SDK 动态库部署到板子。
# 用法（在板子上跑，或 scp 到板子后跑）：
#   sh install_libs.sh [目标库目录，默认 /root/lib]
#
# 然后让程序能找到它们，二选一：
#   A) 临时:  export LD_LIBRARY_PATH=/root/lib:$LD_LIBRARY_PATH
#   B) 永久:  把上面这行加进 /etc/profile，或把 .so 直接拷到 /usr/lib
#
# 这 5 个库的递归依赖已确认闭合（其余 libc/libstdc++/libgcc_s/libz/libatomic
# 是系统库，板子自带）。
#
# 库清单（来自 cvitek_tpu_sdk，编译时所用 SDK）：
#   libcviruntime.so          CVI NPU 推理运行时
#   libcvikernel.so           CVI kernel（被 runtime 依赖）
#   libopencv_core.so.3.2     OpenCV 核心
#   libopencv_imgproc.so.3.2  OpenCV 图像处理
#   libopencv_imgcodecs.so.3.2 OpenCV 编解码

DEST="${1:-/root/lib}"
SRC="$(dirname "$0")/deploy_libs"

if [ ! -d "$SRC" ]; then
    echo "找不到 $SRC，请确认 deploy_libs 和本脚本在一起。"
    exit 1
fi

mkdir -p "$DEST"
cp -aP "$SRC"/* "$DEST"/
echo "已部署到 $DEST :"
ls -l "$DEST"
echo ""
echo "下一步: export LD_LIBRARY_PATH=$DEST:\$LD_LIBRARY_PATH"
echo "或加进 /etc/profile 永久生效。"
