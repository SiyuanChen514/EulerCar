# 使用APT对ADC进行周期采样功能
## 关键字: ADC, APT触发

**【功能描述】**
+ 示例代码基于HAL接口完成时钟、中断和ADC控制器初始化和功能配置。示例中使用APT触发ADC采样，采样结束后触发ADC中断，并在中断回调函数中读取ADC转换结果。

**【示例配置】**
+ ADC触发源：APT。在"SystemInit()”中配置了APT的SOCA可以触发ADC采样。使用接口"HAL_APT_StartModule()”启动APT，在APT的每个周期中都会产生对ADC的触发请求。APT的触发配置请见"System_Init()”内APT初始化的部分。

+ ADC采样源：挂载到SOC0“adcInput”的外部采样源。SOC可以配置为“ADC_SOC_NUM0~ADC_SOC_NUM15”中任何一个。

+ ADC采样结果：ADC中断回调函数中读取结果。中断回调函数的配置请见"System_Init()”内ADC初始化的部分。在回调函数中调用“HAL_ADC_GetConvResult()”获取结果。

**【示例效果】**
+ 当用户烧录编译后的示例代码后，初始化和配置完成后，示例代码中APT会周期性的对ADC进行触发采样。采样成功后，会通过串口打印中断服务函数内读取到的ADC采样结果。在串口打印输出中，会不断打印中断回调函数中读出的ADC采样结果。

**【注意事项】**
+ 示例代码使用UART0进行结果打印输出，需要对UART0进行初始化配置。