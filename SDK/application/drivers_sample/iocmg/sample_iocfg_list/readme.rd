# 使用HAL_IOCMG_Init API完成IO端口的相关功能配置
## 关键字: IOCMG，iomap映射

**【功能描述】**
+ 通过HAL_IOCMG_Init API初始化iolist配置表格。

**【示例配置】**
+ g_ioListTable数组的初始化：该数组中的每个元素值表示一个IO端口的结构体IOCMG_Handle，而每个结构体表示某一个IO端口的所有配置信息（包括端口映射地址值pinTypedef，上下拉pullMode，施密特状态schmidtMode、管脚驱动能力levelShiftRate、电平转换速率driveRate）

**【示例效果】**
+ Debug串口打印配置前的iolist数据，然后打印初始化后的寄存器中的数据，判断配置的数据和预期的是否相同，同时调用HAL_IOCMG_SetXXX API接口和HAL_IOCMG_GetXXX API分别配置和获取IOCMG寄存器的数据是否符合预期。

**【注意事项】**
+ PIN NUMBER和function mode均已编码定义，在iomap中，若配置数据与定义的数据不符则配置失败返回对应的错误，已定义的pin number和function mode可在iomap.h中直接调用。