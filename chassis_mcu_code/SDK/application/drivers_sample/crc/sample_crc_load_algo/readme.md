# 通过改变初始值配置，对同一组数据进行不同CRC算法运算验证
## 关键字: CRC算法，初始值

**【功能描述】**
+ 示例代码基于HAL接口完成时钟、CRC初始化和功能配置。通过load初始值配置，不改变其他配置CRC16_XMODEM算法修改为CRC16_CCIT-FALSE，并对同一组数据进行CRC运算验证。

**【示例配置】**
+ 在"SystemInit()”接口中配置CRC输入数据长度、算法模式等参数。

+ 在进行load初始值配置之前，根据CRC16_XMODEM算法对输入数据0x5678调用“HAL_CRC_SetInputDataGetCheck()”进行CRC计算对比结果。

+ 调用“HAL_CRC_SetCheckInData()”修改初始值0xFFFF,再调用"HAL_CRC_SetInputDataGetCheck()"对输入数据0x5678进行CRC计算对比结果。加载初始值后，CRC16_XMODEM算法修改为CRC16_CCIT-FALSE算法。

**【示例效果】**
+ 当用户烧录编译后的示例代码后，初始化和配置完成后，串口打印原始算法对数据运算生成的CRC数值，并与CRC16_XMODEM算法运算的标准值对比验证正确性；load初始值后，打印load后的初始值，并对同一组数据运算生成CRC数值，并与CRC16_CCIT-FALSE算法运算的标准值对比验证正确性。

**【注意事项】**
+ NA