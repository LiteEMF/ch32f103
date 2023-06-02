# LiteEMF on CH32F103 

本例程在CH32F103官方SDK上移植了LiteEMF框架. 开发者可以在这个基础上开发自己的项目.  
`持续更新中...`

## 关于沁恒

[南京沁恒微电子股份有限公司](https://www.wch.cn/)成立于2004年，是一家接口芯片和全栈MCU芯片公司。

沁恒专注于连接技术和MCU内核研究，基于底层IP研发+芯片设计+底层共性软件开发的全栈研发模式，以自研的处理器IP和收发器IP模块为主，整合构建USB、以太网、蓝牙等接口芯片，及连接型/互联型/无线型全栈MCU+单片机。

技术上涉及“感知+控制+连接+云聚”：ADC模拟检测、MCU智能控制及电机驱动算法、HID人机交互、Ethernet/Bluetooth-LE网络通信、UART/USB/PD/PCI/CAN等接口通讯、物联协议和云端服务，致力于为客户提供万物互联、上下互通的芯片及解决方案。

## CH32F103
沁恒旗下 [CH32F103](https://www.wch.cn/products/CH32F103.html?from=search&wd=eyJpdiI6InBwY0R3MDRJdUpBVXBneGVzbnVqeWc9PSIsInZhbHVlIjoiRW5uNVp3clVHdzUzQU5LS2I3M3JPUT09IiwibWFjIjoiN2Y2ZTUzOGJhZmFlYWEyMzEyNDlmMjM5NGJiZDViZjQ4ZWVjNGE2ZTAxMjFlNTBjZTFkYzAwOGE1MTQ3YTE4YiJ9) 32位通用增强型Cortex-M3单片机:

CH32F1系列是基于32位Cortex-M3内核设计的通用微控制器。片上集成了时钟安全机制、多级电源管理、 通用DMA控制器等。 此系列具有 2 路 USB2.0接口、多通道 TouchKey、 12 位 DAC 转换模块，多通道 12 位 ADC、 多组定时器、 CAN 通讯控制器、 I2C/USART/SPI 等丰富的外设资源。


## LiteEMF适配
 
 在原厂sdk V1.8基础上适配了LiteEMF框架, emf_ch32fxxx_hal/目录下是CH32F103的hal驱动, project/ 目录下为工程文件基于FreeRTOS/demo上修改.

 * ADC, IIC, SPI, 都适配了DMA/isr中断方式节约CPU
 * tick 使用的是FreeRTOS系统的tick时钟适配
 * mem动态内存适配的是FreeRTOS系统自带的heap_4
 * usb驱动目前进行中还未完成
 * DFU升级功能呢目前还未完成


## 快速开始

* 下载项目 `git clone https://github.com/LiteEMF/ch32f103`
* 安装 keil5, 安装[Keil.WCH32F1xx_DFP.1.0.1.pack](/Keil.WCH32F1xx_DFP.1.0.1.pack)芯片支持
* keil 打开 ch32f103\project\project.uvprojx 编译
* 通过下载工具[WCHISPTool_Setup.exe](https://www.wch.cn/downloads/WCHISPTool_Setup_exe.html) usb/uart下载
* hw_confg.h,hw_board.h文件添加或修改自己的硬件和工程


