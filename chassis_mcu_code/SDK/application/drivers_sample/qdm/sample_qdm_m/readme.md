# M法测量电机实时转速
## 关键字: QDM, 电机，M法

**【功能描述】**
+ 通过获取电机编码器QDM的相关信息，使用M法进行计算电机实时转速

**【示例配置】**
+ QDM控制配置：解码模式decoderMode，极性选择polarity，正交分解resolution、触发锁存模式trgLockMode，A/B相互换swap，PTU单元周期模式ptuMode

+ 输入滤波配置：可通过“g_qdmHandle.inputFilter”对A/B/Z相滤波等级进行配置，默认均为0

+ PPU电机位置处理单元配置：位置计数模式pcntMode，位置计数复位模式pcntRstMode，位置计数初始化模式pcntIdxInitMode，位置计数器最大值posMax，位置计数初始化值posInit

+ TSU时间戳单元配置：TSU计数器最大值qcMax，TSU预分频tsuPrescaler

+ PTU周期触发单元配置：PTU周期大小period

+ 其他配置：捕获事件预分频cevtPrescaler，编码器线数motorLineNum，中断使能interruptEn，子模块使能subModeEn

**【示例效果】**
+ 串口打印出电机实时的转速值大小

**【注意事项】**
+ 需要转动电机才能进行测量实时转速
