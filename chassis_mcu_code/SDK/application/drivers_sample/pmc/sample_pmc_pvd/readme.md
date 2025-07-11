# 配置PMC模块的PVD功能，实现对VDD电压的监测
## 关键字: PMC, PVD，中断, 电压监测

**【功能描述】**
+ 示例代码基于HAL接口完成时钟、PMC控制器初始化和功能配置。通过PVD实现对VDD电压的监测，PVD中断发生时Debug串口打印相关提示信息。

**【示例配置】**

+ PMC初始化：调用接口"HAL_PMC_Init()”完成对示例代码中PMC的基地址、唤醒模式、唤醒时间或唤醒触发类型、PVD阈值进行配置，其中PVD阈值可以选择的范围是PMC_PVD_THRED_LEVEL2 - PMC_PVD_THRED_LEVEL7。

+ 调用"HAL_PMC_RegisterCallback()"注册PVD中断回调函数，并使能相关中断配置。

**【示例效果】**
+ 当用户烧录编译后的示例代码后，初始化和配置完成后，示例代码中在监测到VDD电压高于或低于设定的PVD阈值时，会产生PVD中断并通过Debug串口打印"PVD happen!"提示信息。

**【注意事项】**
+ 示例代码使用UART0进行结果打印输出，需要对UART0配置。