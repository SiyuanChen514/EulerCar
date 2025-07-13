# CAPM读取电机HALL位置传感器
## 关键字: CAPM, HALL传感器

**【功能描述】**
+ 使用三个CAPM捕获三个HALL传感器的电平信息

**【示例配置】**
+ 捕获模式：可通过“g_capmAConfig.capMode”进行配置，默认为连续捕获CAPM_CONTINUECAP

+ 预分频：对CAPM输入信号进行预分频，可通过”g_capmAConfig.preScale“进行配置，默认为不分频

+ 捕获寄存器配置：可通过”g_capmAConfig.capRegConfig“进行配置，默认为上升沿捕获，每次复位

**【示例效果】**
+ 在IOCMG_15、IOCMG_18和IOCMG_19输入HALL传感器的输入信号。在串口打印的数据中的每一位表示当前时刻的对应的HALL传感器的电平状态。

**【注意事项】**
+ 需要转动电机才能进行CAPM捕获霍尔传感器信号
