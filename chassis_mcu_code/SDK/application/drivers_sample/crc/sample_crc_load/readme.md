# 通过改变初始值配置，对同一组数据进行不同CRC算法运算验证
## 关键字: CRC算法，初始值

**【功能描述】**
+ 示例代码基于HAL接口完成时钟、中断和CRC初始化和功能配置。通过load初始值配置，不改变其他配置CRC16_XMODEM算法修改为CRC16_CCIT-FALSE，并对同一组数据进行CRC运算验证。

**【示例配置】**
+ 在"SystemInit()”接口中配置CRC输入数据长度、算法模式、反压超时时间、开启反压中断等参数。

+ 在进行load初始值配置之前，根据CRC16_XMODEM算法对输入数据0x5678调用“HAL_CRC_SetInputDataGetCheck()”进行CRC计算对比结果。

+ 调用“HAL_CRC_SetCheckInData()”修改初始值0xFFFF,再调用"HAL_CRC_SetInputDataGetCheck()"对输入数据0x5678进行CRC计算对比结果。加载初始值后，CRC16_XMODEM算法修改为CRC16_CCIT-FALSE算法。

+ 配置反压中断的回调函数"CRC_CallbackFunc()"，当产生反压中断且中断使能时调用。

**【示例效果】**
+ 当用户烧录编译后的示例代码后，初始化和配置完成后，串口打印原始算法对数据运算生成的CRC数值，并与CRC16_XMODEM算法运算的标准值对比验证正确性；load初始值后，打印load后的初始值，并对同一组数据运算生成CRC数值，并与CRC16_CCIT-FALSE算法运算的标准值对比验证正确性。如果产生CRC计算反压超时则产生中断，调用反压中断回调函数，打印CRC反压超时。

**【注意事项】**
+ 目前CRC算法只支持CRC8_ROHC、CRC16_IBM、CRC16_MODBUS、CRC16_CCITT_FALSE、CRC16_XMODEM、CRC32算法，且只有算法初始值可配置，其余配置项均已硬件固化。