# UART-Tx和Rx在DMA模式下同时收发数据
## 关键字: UART, Tx_DMA_Rx_INT收发数据

**【功能描述】**
+ UART_DMA全双工收发数据，UART_DMA写的过程中，可以进行UART_DMA读。

**【示例配置】**
+ 预设置的缓存：配置发送数据和接收数据缓存地址

+ UART的Tx和Rx都配置为DMA模式。

+ UART_DMA发送数据：将待发送首地址和字符长度作为入参传入HAL_UART_WriteDMA()。

+ UART_DMA接收数据：将存放接收的首地址，和接收长度作为入参传入到HAL_UART_ReadDMA()中。

+ 串口0会打印发送和接收的数据。

**【示例效果】**
+ 示例程序会不断的进行UART_DMA发送、接收，实现UART_DMA写的过程中，可以进行UART_DMA读。

**【注意事项】**
+ 串口通信的波特率，校验位等配置需保持一致，可以通过UART配置界面进行更改。