# STM32 TCRT5000 串口测试烧录说明

## 目的

本测试程序只读取两个红外传感器并通过 USART1 输出原始电平，不初始化电机、编码器、灯带和防跌落运动逻辑，不会驱动车轮。

## 引脚

- PA11：左侧 TCRT5000
- PA12：右侧 TCRT5000
- USART1：PA9 TX、PA10 RX，115200，8-N-1

## 工程操作

1. 在 Keil 工程中将原 USER/main.c 从工程组移除或排除编译。
2. 将 USER/main_tcrt_test.c 加入工程，并把它设置为当前编译的主程序文件；工程中只能保留一个 main()。
3. 保留 SYSTEM/usart/usart.c、串口头文件、系统时钟和 delay 相关文件。
4. 可以移除或排除 motor、encoder、control_system、colorful_led、cliff_sensor 等原运动模块；本测试主程序不引用它们。
5. 编译、下载后打开串口监视器，参数设置为 115200/8/N/1。

## 预期日志

TCRT TEST START

PA11=LEFT PA12=RIGHT BAUD=115200

随后每 500 ms 输出：

TCRT L=0 R=1

其中 L/R 是 PA11/PA12 的原始数字电平，先记录灰色大理石、左黑右灰、左灰右黑、双黑和悬空五种状态。当前不对 0/1 做黑白解释，最终逻辑以实测为准。

## 乱码排查

如果仍然乱码，先确认：

- 串口工具是 115200，而不是 9600；
- 监视的是 STM32 USART1 对应的 USB 通道；
- SW2 拨到了 STM32/MCU 通道，而不是 Hi3861 通道；
- USB 转串口 TX/RX 没有接反；
- USB 转串口与 STM32 共地；
- 下载后重新打开串口，避免复位前残留数据；
- 电源稳定且没有同时运行 Hi3861 对 UART 的占用。
