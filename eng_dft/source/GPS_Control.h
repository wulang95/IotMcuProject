#ifndef  __GPS_CONTROL_H
#define  __GPS_CONTROL_H

#include <stdio.h>
#include <string.h>
#include <stddef.h>
#include <stdint.h>
#include <assert.h>

#pragma pack(1)
typedef struct GPS_DATA_STRUCT
{
    char     Time1[12];          		//定位时间1 时分秒 hhmmss.00
    uint8_t  GpsFlag;            		//获取到GPS数据标志
    char  GPSValidFlag;       		//有效定位标志      0:无有效定位 1:有有效定位
	

	//2023-4-7 19:06:35  兼容29H，原先是32字节，改为48
    char     LatLongData[48];    		//经纬度信息  ddmm.mmmm,N/S,dddmm.mmmm,E/W  2238.07773,N,11407.55384,E  

    char     SateNumStr[6];      		//GPS卫星数量
    char     HDOP[6];            		//水平精度因子  0.5~99.99
    char     Time2[12];          		//定位时间1 年月日 ddmmyy
    char     SeaLevelH[12];      		//海平面高度  (-9999.9 - 9999.9),M
    char     Mode[3];            		//定位模式  A=自主定位,D=差分,E=估算,N=无效

    uint8_t  SolType;            		//当前定位类型
    double   Latitude;           		//纬度数据  小数格式
    double   Longitude;          		//经度数据  小数格式

	  uint8_t  RefreshFlag;        //定位刷新标志
} GPS_DATA;
#pragma pack()
void GPS_Control();
void GPS_init();

void GPS_host_start_cmd();
void GPS_deep_sleep_cmd();
void GPS_power_on();
void GPS_power_off();
void GPS_data_task();



#endif
