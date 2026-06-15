编译：TPU_SDK_PATH=/workspace/cvitek_tpu_sdk OPENCV_PATH=/workspace/cvitek_tpu_sdk/opencv USE_UVC=1 bash build_riscv_debug.sh
把tennis和S97pinmux.sh、训练好的模型放入rootfs同一路径
```
./S97pinmux.sh start
./tennis tennis.cvimodel 0 /dev/ttyS1 /dev/ttyS2 -25 //最后一个-25是球的位置
```
