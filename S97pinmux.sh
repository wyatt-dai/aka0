#!/bin/sh
#
# S97pinmux - Boot-time UART pinmux setup (SG2002 / LicheeRV Nano)
#
# 必须在 S98tennis 之前运行（编号 97 < 98），否则 tennis 打开串口时
# 引脚还停在默认功能上，发帧没反应。
#
# 引脚分配（硬件沿用 rk3588，物理设备没变，只是换了 2002 板子）：
#   电机   ESP32-C3 底盘 -> ttyS1，引到 GPIOA18/A19（官方双口并用配法）
#   机械臂 ZP10D 舵机    -> ttyS2，引到 GPIOA28/A29
#
# 寄存器值来源：Sipeed wiki + dts 实证，见 memory/sg2002-uart-pinmux-mapping。
# A28/A29 上电默认 0x1=UART1，这里切 0x2=UART2 给机械臂；
# 电机改走 A18/A19 的 UART1 复用（0x6），两口互不抢脚。

start() {
    echo "[pinmux] Configuring UART pinmux..."

    # 电机 ttyS1 -> GPIOA18/A19
    devmem 0x03001068 32 0x6   # GPIOA18 -> UART1 RX
    devmem 0x03001064 32 0x6   # GPIOA19 -> UART1 TX

    # 机械臂 ttyS2 -> GPIOA28/A29
    devmem 0x03001070 32 0x2   # GPIOA28 -> UART2 TX
    devmem 0x03001074 32 0x2   # GPIOA29 -> UART2 RX

    echo "[pinmux] Done: 电机=ttyS1(A18/A19)  机械臂=ttyS2(A28/A29)"
}

stop() {
    # pinmux 没有"停"的概念，留空保持 init 接口一致
    echo "[pinmux] (nothing to stop)"
}

case "$1" in
    start)
        start
        ;;
    stop)
        stop
        ;;
    restart)
        start
        ;;
    *)
        echo "Usage: $0 {start|stop|restart}"
        exit 1
        ;;
esac

exit 0
