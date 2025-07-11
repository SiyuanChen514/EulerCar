# UART阻塞模式发送数据
## 关键字: UART, 阻塞模式发送

**【功能描述】**
+ 在阻塞下模式下，阻塞一段时间等待UART发送数据。

+ 阻塞模式下的发送数据的超时时间为2000ms。

**【示例配置】**
+ 待发送数据：将待发送首地址和字符长度作为入参传入HAL_UART_WriteBlocking()。

+ 发送超时时间配置：通过“HAL_UART_WriteBlocking()”进行配置，单位为ms。

+ HAL_UART_WriteBlocking()返回值BASE_STATUS_OK，表示在给定时间内，发送完成数据；返回值为BASE_STATUS_TIMEOUT，表示在规定时间内，未能完成数据发送；其他返回值，表示错误。

+ 串口0会打印发送成功或发送超时。

**【示例效果】**
+ 串口0发送数据，若在超时时长内发送数据，则打印“Send success!”，若发送超时则打印“Send time out!”， 发送错误，打印“Send verification error!”。

**【注意事项】**
+ 串口通信的波特率，校验位等配置需保持一致，可以通过UART配置界面进行更改。