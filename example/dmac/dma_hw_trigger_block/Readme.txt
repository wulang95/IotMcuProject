================================================================================
                                样例使用说明
================================================================================
版本历史 
日期        版本    负责人         IAR     MDK         描述
2018-05-14  0.1     WeiPeng        7.70    5.26       first version
================================================================================
功能描述
================================================================================
1. 本样例主要展示DMA block传输功能在ADC连续扫描采样中的应用。
2. 每执行，但DMA只读取一个BlockSize的数据量，此样例中BlockSize=3。
3. 执行第一个Adc_SQR_Start()，DMA读取SQRRESULT0 SQRRESULT1 SQRRESULT2 
4. 执行第二个Adc_SQR_Start()，DMA读取SQRRESULT3 SQRRESULT4 SQRRESULT5 
5. 执行第三个Adc_SQR_Start()，DMA读取SQRRESULT6 SQRRESULT7 SQRRESULT8 
================================================================================
测试环境
================================================================================
测试用板:
---------------------
HC32LF07x_EVB

辅助工具:
---------------------
示波器
电源

辅助软件:
---------------------
无

================================================================================
使用步骤
================================================================================
1）打开工程编译并运行
2）在三个ADC输入PIN(PA00 PA02 PA05)输入不同的电压值。
3）在调试窗口中观察数组ADC_Result_Array的值
4）判断是否每执行一次Adc_SQR_Start()，DMA传输三个数据到ADC_Result_Array数组中
5）根据PA00 PA02 PA05输入的电压值，判断ADC_Result_Array数组中的值是否合理，如果
在合理范围内，则运行正常


================================================================================
注意
================================================================================

================================================================================
