================================================================================
                                样例使用说明
================================================================================
版本历史 
日期        版本    负责人         IAR     MDK   描述
2018-05-20  0.1     LUX          7.70      5.26  first version
================================================================================
功能描述
================================================================================
本样例主要系统时钟监测功能。
说明：
本样例使用RC10K时钟对外部晶振XTL 32768Hz校准。

================================================================================
测试环境
================================================================================
测试用板:
---------------------
HC32LF07x_EVB

辅助工具:
---------------------


辅助软件:
---------------------

================================================================================
使用步骤
================================================================================
1）打开工程并重新编译；
2）启动IDE的下载和调试功能；
3）运行，按下USER KEY后开始执行时钟监测流程；
4）测试可采用将外部晶振短接，或其他合理方式使外部低速晶振停振或异常，（可用示波器辅助观测）
4）运行结束后观察u8TrimTestFlag的值，若为FF则表示监测到时钟异常；

================================================================================
注意
================================================================================
实际使用时，该功能可用于监测外部时钟状态，并在监测到异常时及时采取相应的处理动作。
================================================================================
