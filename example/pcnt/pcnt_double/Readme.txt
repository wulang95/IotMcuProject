================================================================================
                                样例使用说明
================================================================================
版本历史 
日期        版本    负责人         IAR     MDK   描述
2018-05-03  0.1                    7.70    5.26  first version
|
================================================================================
功能描述
================================================================================
本样例主要展示PCNT 正交编码计数功能。
说明：
本样例对展示PCNT 正交编码计数产生中断，当计数从设定值递减到0时再加1即产生下溢中断，
并改变板上蓝色LED的亮/灭状态

================================================================================
测试环境
================================================================================
测试用板:
---------------------
HC32LF07x_EVB

辅助工具:
---------------------
无

辅助软件:
---------------------
无。

================================================================================
使用步骤
================================================================================
1）打开工程并重新编译；
2）启动IDE的下载和调试功能；
3）外部输入信号到PB05和PB07；
4）运行；
5）观察LED信号线与输入脉冲关系，输入一个脉冲信号，计数一次同时LED灯翻转闪烁。

================================================================================
注意
PB05接滞后90°的脉冲输入
PB07接比PB05超前90°的脉冲输入
脉冲输入频率为200Hz，占空比为50%
================================================================================

================================================================================
