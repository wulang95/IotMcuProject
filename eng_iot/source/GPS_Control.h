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
    char     Time1[12];          		//��λʱ��1 ʱ���� hhmmss.00
    uint8_t  GpsFlag;            		//��ȡ��GPS���ݱ�־
    char  GPSValidFlag;       		//��Ч��λ��־      0:����Ч��λ 1:����Ч��λ
	

	//2023-4-7 19:06:35  ����29H��ԭ����32�ֽڣ���Ϊ48
    char     LatLongData[48];    		//��γ����Ϣ  ddmm.mmmm,N/S,dddmm.mmmm,E/W  2238.07773,N,11407.55384,E  

    char     SateNumStr[6];      		//GPS��������
    char     HDOP[6];            		//ˮƽ��������  0.5~99.99
    char     Time2[12];          		//��λʱ��1 ������ ddmmyy
    char     SeaLevelH[12];      		//��ƽ��߶�  (-9999.9 - 9999.9),M
    char     Mode[3];            		//��λģʽ  A=������λ,D=���,E=����,N=��Ч

    uint8_t  SolType;            		//��ǰ��λ����
    double   Latitude;           		//γ������  С����ʽ
    double   Longitude;          		//��������  С����ʽ

	  uint8_t  RefreshFlag;        //��λˢ�±�־
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
