# UART检测特定字符
## 关键字: UART, 特定字符检测

**【功能描述】**
+ UART检测特定字符，检测到特定字符，会触发检测成功回调函数。

**【示例配置】**
+ 特定字符检测：示例中设定的检测字符为“A”，当收到带有字符“A”的数据，UART就会触发字符检测回调函数。用户可以使用“HAL_UART_OpenCharacterMatchEx”函数来设定需要检测的字符。

+ 字符检测成功中断：字符检测成功之后，会以中断的形式调用字符检测成功函数“UART_CharacterMatchCallBack”，用户可以使用“HAL_UART_RegisterCallBack”进行更改。

**【示例效果】**
+ 向串口0发送大写字符"123Acharactermatch"，如果串口输出“Match character success”字符，则代表检测到字符“A”。

**【注意事项】**
+ 串口通信的波特率，校验位等配置需保持一致，可以通过UART配置界面进行更改。