================================================================================
                                样例使用说明
================================================================================
版本历史 
日期        版本    负责人         IAR     MDK   描述
2018-05-03  0.1                    7.70    5.26  first version
================================================================================
功能描述
================================================================================
本样例主要展示PCA的捕获功能。
说明：
本样例对展示PCA对外部上升/下降沿的捕获功能，捕获到沿时产生中断。

================================================================================
测试环境
================================================================================
测试用板:
---------------------
HC32LF07x_EVB

辅助工具:
---------------------
示波器

辅助软件:
---------------------
无。

================================================================================
使用步骤
================================================================================
1）打开工程并重新编译；
2）启动IDE的下载和调试功能；
3）用跳线连接PA07（CAP1)与PA06(为PCA通道0输出的8位PWM)，可以通过示波器观察PA06的输出；
4）运行；
5）程序将检测上升沿和下降沿：
PCA通道0(PA06:输出8位PWM)->PCA通道1(PA07:接收上升沿和下降沿)


================================================================================
注意
================================================================================

================================================================================
