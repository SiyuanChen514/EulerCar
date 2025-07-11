# 验证CMM时钟频率监测功能，同时验证CMM频率错误中断功能
## 关键字: CMM, 时钟频率监测, 中断

**【功能描述】**
+ 用于检查目标时钟（LOSC/HOSC/TCXO/HS_CLK/LS_CLK）频率是否发生错误。每隔5s分别注入频率错误中断，用于验证CMM的频率中断处理函数。

**【示例配置】**
+ CMM参考时钟源：参考时钟源可通过“g_cmm.refClockSource”配置，默认为内部低速时钟CMM_REF_CLK_LOSC，分频比可通过“g_cmm.refFreqDivision”配置，默认为不分频CMM_REF_FREQ_DIV_0。

+ CMM目标时钟源：目标时钟源可通过“g_cmm.targetClockSource”配置，默认为内部低速时钟CMM_TARGET_CLK_LOSC，分频比可通过“g_cmm.targetFreqDivision”配置，默认为8192分频CMM_TARGET_FREQ_DIV_8192。

+ CMM上下限值： 上限值可通过“g_cmm.upperBound”配置，下限值可通过“g_cmm.lowerBound”配置。

+ CMM中断类型：中断类型可通过“g_cmm.interruptType”进行配置，默认为频率错误中断。

**【示例效果】**
+ Debug串口首先每隔1s打印出监测目标时钟频率的CMCNTLOCK锁存值，在注入频率错误中断后每隔1s分别打印频率错误回调函数中的log, 5s后解除频率错误中断后再每隔1s打印CMCNTLOCK锁存值，循环往复。

**【注意事项】**
+ 上下限值的范围可反应系统对目标时钟偏差的冗余度，可自行设计门限值和时钟分频比，门限值具体计算方法请参照芯片技术指南。
