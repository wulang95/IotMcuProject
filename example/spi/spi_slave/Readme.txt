================================================================================
                                样例使用说明
================================================================================
版本历史 
日期        版本    负责人         IAR     MDK   描述
2019-09-24  0.2       Lux          7.70    5.26  更新SPI操作流程
================================================================================
功能描述
================================================================================
本样例主要展示SPI初始化以及从机模式发送和接收功能。
说明：
    

================================================================================
测试环境
================================================================================
测试用板:
---------------------
HC32LF07x_EVB

辅助工具:
---------------------
电源

辅助软件:
---------------------
无

================================================================================
使用步骤
================================================================================
1）打开工程编译
2）连接两块STK板，与从机接口连线正确
3）运行程序，板上LED亮表示接收到的数据与tx_buf[10]相同。
4）SPI 相应的引脚设置
    SPI1(从机)
    PB12:CS
    PB14:MISO
    PB15:MOSI
    PB13:SCK
================================================================================
注意
================================================================================
与例程spi_master联调
================================================================================
