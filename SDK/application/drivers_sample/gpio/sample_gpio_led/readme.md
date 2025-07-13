# 配置GPIO管脚的电平反转功能，实现GPIO管脚控制LED灯的亮灭
## 关键字: GPIO, 电平反转，LED

**【功能描述】**
+ 示例代码基于HAL接口完成时钟、GPIO控制器初始化和功能配置。在示例代码中通过控制GPIO管脚的电平翻转实现控制LED灯的亮灭控制。

**【示例配置】**
+ GPIO管脚选择：示例代码中选择GPIO管脚用于控制LED灯亮灭。也可以选择其他GPIO管脚用于控制LED的功能，在"GPIO_Init()"接口中的"g_gpiox.baseAddress"可以配置其它GPIOX，g_gpiox.pins可以配置“GPIO_PIN_0-GPIO_PIN_7”中的任意一个。
  
+ GPIO管脚初始化：调用接口"HAL_GPIO_Init()”完成对示例代码中GPIO管脚的方向、电平、中断模式配置。

+ GPIO管脚实现对LED灯控制：对于输出管脚调用接口"HAL_GPIO_TogglePin()"实现管脚电平翻转，结合延时函数实现GPIO管脚控制LED灯每50ms亮灭状态反转一次。

**【示例效果】**
+ 当用户烧录编译后的示例代码后，初始化和配置完成后，示例代码中会控制GPIO管脚连接的LED每50ms进行一次亮灭，每反转一次电平Debug串口会打印一条信息。

**【注意事项】**
+ 示例代码使用UART0进行结果打印输出，需要对UART0配置。
+ 示例代码中使用的GPIO管脚需和LED灯连接在一起。