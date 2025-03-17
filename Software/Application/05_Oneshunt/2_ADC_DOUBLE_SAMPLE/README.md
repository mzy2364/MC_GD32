# Motor_Control_GD32_ADC_DOUBLE_SAMPLE

基于 Motor_Control_GD32 电机开发板的单电阻 FOC 工程

本例程演示 GD32 的 PWM 移相和 ADC 双采样功能，其中 V 相的占空比固定为 50% 而且不移相，U 相向左移动，W 相向右移动，用 U 相和 V 相分别触发 ADC 进行采样

测试过程中不需要接实际的电机

USER_GPIO3 的脉冲用于指示 ADC 采样完成的中断信号

用示波器测量 UL VL WL 信号可以测试 PWM 的移相功能是否正常

也可以用示波器直接测量 U V W 三相信号来测试 PWM 的移相功能是否正常