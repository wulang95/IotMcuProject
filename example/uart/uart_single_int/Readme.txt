================================================================================
                                样例使用说明
================================================================================
版本历史 
日期        版本    负责人         IAR     MDK   描述
2019-04-23  0.1     CW             7.70    5.16 first version
================================================================================
功能描述
================================================================================
本样例主要展示串口单线半双工收发数据功能

说明：
1. 配置UART0为主机，UART1为从机。
2. 连线方式如下：
   PA09：UART0_TX <----------> PA02：UART1_TX
3. 配置波特率9600bps
4. 此功能在单板上运行

================================================================================
测试环境
================================================================================
测试用板:
HC32L136_STK
---------------------
（需要填）

辅助工具:
---------------------
杜邦线
固件下载器

辅助软件:
---------------------


================================================================================
使用步骤
1. 用杜邦线连接PA09与PA02引脚。
2. 打开工程编译并运行
2、在调试窗口中在检查数组uart0_u8RxData的值是否为0x33、0x44，
检查数组uart1_u8RxData的值是否为0x11、0x22，如果正确，则运行正常。
================================================================================

================================================================================
注意：
================================================================================
无
================================================================================
