/******************************************************************************
* Copyright (C) 2019, Xiaohua Semiconductor Co.,Ltd All rights reserved.
*
* This software is owned and published by:
* Xiaohua Semiconductor Co.,Ltd ("XHSC").
*
* BY DOWNLOADING, INSTALLING OR USING THIS SOFTWARE, YOU AGREE TO BE BOUND
* BY ALL THE TERMS AND CONDITIONS OF THIS AGREEMENT.
*
* This software contains source code for use with XHSC
* components. This software is licensed by XHSC to be adapted only
* for use in systems utilizing XHSC components. XHSC shall not be
* responsible for misuse or illegal use of this software for devices not
* supported herein. XHSC is providing this software "AS IS" and will
* not be responsible for issues arising from incorrect user implementation
* of the software.
*
* Disclaimer:
* XHSC MAKES NO WARRANTY, EXPRESS OR IMPLIED, ARISING BY LAW OR OTHERWISE,
* REGARDING THE SOFTWARE (INCLUDING ANY ACOOMPANYING WRITTEN MATERIALS),
* ITS PERFORMANCE OR SUITABILITY FOR YOUR INTENDED USE, INCLUDING,
* WITHOUT LIMITATION, THE IMPLIED WARRANTY OF MERCHANTABILITY, THE IMPLIED
* WARRANTY OF FITNESS FOR A PARTICULAR PURPOSE OR USE, AND THE IMPLIED
* WARRANTY OF NONINFRINGEMENT.
* XHSC SHALL HAVE NO LIABILITY (WHETHER IN CONTRACT, WARRANTY, TORT,
* NEGLIGENCE OR OTHERWISE) FOR ANY DAMAGES WHATSOEVER (INCLUDING, WITHOUT
* LIMITATION, DAMAGES FOR LOSS OF BUSINESS PROFITS, BUSINESS INTERRUPTION,
* LOSS OF BUSINESS INFORMATION, OR OTHER PECUNIARY LOSS) ARISING FROM USE OR
* INABILITY TO USE THE SOFTWARE, INCLUDING, WITHOUT LIMITATION, ANY DIRECT,
* INDIRECT, INCIDENTAL, SPECIAL OR CONSEQUENTIAL DAMAGES OR LOSS OF DATA,
* SAVINGS OR PROFITS,
* EVEN IF Disclaimer HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
* YOU ASSUME ALL RESPONSIBILITIES FOR SELECTION OF THE SOFTWARE TO ACHIEVE YOUR
* INTENDED RESULTS, AND FOR THE INSTALLATION OF, USE OF, AND RESULTS OBTAINED
* FROM, THE SOFTWARE.
*
* This software may be replicated in part or whole for the licensed use,
* with the restriction that this Disclaimer and Copyright notice must be
* included with each copy of this software, whether used in part or whole,
* at all times.
*/
/******************************************************************************/
/** \file s25fl132k_spi_flash.c
 **
 ** A detailed description is available at
 ** @link Sample Group Some description @endlink
 **
 **   - 2019-08-08   linsq   First version
 **
 ******************************************************************************/

/******************************************************************************
 * Include files
 ******************************************************************************/
   
#include "s25fl132k_spi_flash.h"   
#include <stdio.h>   
#include "gpio.h"
#include "spi.h"
/**
 ******************************************************************************
 ** \brief  S25FL132K_SPI_FLASH_CS_LOW����CSΪ�͵�ƽ
 **
 ** \param  ��
 ** \return ��
 ******************************************************************************/
void S25FL132K_SPI_FLASH_CS_LOW(void)
{
    Spi_SetCS(M0P_SPI0,FALSE);
}
  
/**
 ******************************************************************************
 ** \brief  ʹ��SPI1_CS�ߵ�ƽ
 **
 ** \param  ��
 ** \return ��
 ******************************************************************************/
void S25FL132K_SPI_FLASH_CS_HIGH(void)
{
    Spi_SetCS(M0P_SPI0,TRUE);
}

/**
 ******************************************************************************
 ** \brief  ��ʼ��SPI0��Ӧ������
 **
 ** \param  ��
 ** \return ��
 ******************************************************************************/
void GPIO_Config(void)
{
    stc_gpio_cfg_t GpioInitStruct; 
    //����PE12ΪSPI0_CS
    GpioInitStruct.enDir = GpioDirOut;                 //���ó����
    Gpio_Init(GpioPortE, GpioPin12, &GpioInitStruct); 
    Gpio_SetAfMode(GpioPortE, GpioPin12, GpioAf2);     //�˿�ѡ����ΪSPI0_CS����
    //����PE13ΪSPI0_SCK
    GpioInitStruct.enDir = GpioDirOut;                 //���ó����
    Gpio_Init(GpioPortE, GpioPin13, &GpioInitStruct); 
    Gpio_SetAfMode(GpioPortE, GpioPin13, GpioAf2);     //�˿�ѡ����ΪSPI0_SCK����
    //����PE14ΪSPI0_MISO
    GpioInitStruct.enDir = GpioDirIn;                  //���ó�����
    Gpio_Init(GpioPortE, GpioPin14, &GpioInitStruct); 
    Gpio_SetAfMode(GpioPortE, GpioPin14, GpioAf2);     //�˿�ѡ����ΪSPI0_MISO����
    //����PE15ΪSPI0_MOSI
    GpioInitStruct.enDir = GpioDirOut;                 //���ó����
    Gpio_Init(GpioPortE, GpioPin15, &GpioInitStruct); 
    Gpio_SetAfMode(GpioPortE, GpioPin15, GpioAf2);     //�˿�ѡ����ΪSPI0_MOSI����      
}

/**
 ******************************************************************************
 ** \brief  ��ʼ��SPI0���������
 **
 ** \param  ��
 ** \return ��
 ******************************************************************************/
void SPI_Config(void)
{
    stc_spi_cfg_t  SpiInitStruct;
    Sysctrl_SetPeripheralGate(SysctrlPeripheralSpi0, TRUE);//ʹ��SPI0ģ���ʱ��
    SpiInitStruct.enCPHA = SpiMskCphasecond;//ʱ��ѡ��ڶ�����
    SpiInitStruct.enCPOL = SpiMskcpolhigh;//ʱ�Ӽ���ѡ�񣺸�
    SpiInitStruct.enPclkDiv = SpiClkMskDiv4;//������ѡ��fsys/4
    SpiInitStruct.enSpiMode = SpiMskMaster;//���ô�����ģʽ
    Spi_Init(M0P_SPI0, &SpiInitStruct);
    Spi_SetCS(M0P_SPI0,TRUE);         //CS0�ߵ�ƽ    
}

/**
 ******************************************************************************
 ** \brief  ��ʼ��SPI1������������
 **
 ** \param  ��
 ** \return ��
 ******************************************************************************/
void S25FL132K_SPI_FLASH_Init(void)   
{   
    GPIO_Config();
    SPI_Config();
}   
   
/**
 ******************************************************************************
 ** \brief  ��������
 **
 ** \param  SectorAddr: ��Ҫ��������������ʼ��ַ
 ** \return ��
 ******************************************************************************/   
void S25FL132K_SPI_FLASH_SectorErase(uint32_t SectorAddr)   
{   
    SectorAddr = SectorAddr<<12;
    S25FL132K_SPI_FLASH_WriteEnable();                           //дʹ�� 
    S25FL132K_SPI_FLASH_WaitForWriteEnd();                       //�ȴ��������
    S25FL132K_SPI_FLASH_CS_LOW();                                //CS����͵�ƽ  
    S25FL132K_SPI_FLASH_SendByte(S25FL132K_SectorErase);         //���Ͳ�������ָ��  
    S25FL132K_SPI_FLASH_SendByte((SectorAddr & 0xFF0000) >> 16); //����Ҫ������������ַ��16bit-23bit   
    S25FL132K_SPI_FLASH_SendByte((SectorAddr & 0xFF00) >> 8);    //����Ҫ������������ַ��8bit-15bit  
    S25FL132K_SPI_FLASH_SendByte(SectorAddr & 0xFF);             //����Ҫ������������ַ��0bit-7bit    
    S25FL132K_SPI_FLASH_CS_HIGH();                               //CS����ߵ�ƽ  
    S25FL132K_SPI_FLASH_WaitForWriteEnd();                       //�ȴ��������
}   
   
/**
 ******************************************************************************
 ** \brief  ������Ƭ
 **
 ** \param  ��
 ** \return ��
 ******************************************************************************/  
void S25FL132K_SPI_FLASH_BulkErase(void)   
{   
    S25FL132K_SPI_FLASH_WriteEnable();                           //����дʹ��  
    S25FL132K_SPI_FLASH_CS_LOW();                                //CS����͵�ƽ  
    S25FL132K_SPI_FLASH_SendByte(S25FL132K_ChipErase);           //���Ͳ�����Ƭ��ָ��  
    S25FL132K_SPI_FLASH_CS_HIGH();                               //CS����ߵ�ƽ 
    S25FL132K_SPI_FLASH_WaitForWriteEnd();                       //�ȴ��������   
}   
     
/**
 ******************************************************************************
 ** \brief  ҳ��̣�д�뷶ΧΪһҳ��С����
 **
 ** \param  pBuffer:ָ��Ҫд��ҳ������ָ��
 ** \param  WriteAddr:Ҫд���ݵ�FLASHĿ���ַ
 ** \param  NumByteToWrite:Ҫд��ҳ�������ֽ���
 ** \return ��
 ******************************************************************************/ 
void S25FL132K_SPI_FLASH_PageWrite(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite)   
{     
    S25FL132K_SPI_FLASH_WriteEnable();                           //����дʹ��      
    S25FL132K_SPI_FLASH_CS_LOW();                                //CS����͵�ƽ     
    S25FL132K_SPI_FLASH_SendByte(S25FL132K_PageProgram);         //����ҳ���ָ��    
    S25FL132K_SPI_FLASH_SendByte((WriteAddr & 0xFF0000) >> 16);  //����Ҫ������������ַ��16bit-23bit  
    S25FL132K_SPI_FLASH_SendByte((WriteAddr & 0xFF00) >> 8);     //����Ҫ������������ַ��8bit-15bit  
    S25FL132K_SPI_FLASH_SendByte(WriteAddr & 0xFF);              //����Ҫ������������ַ��0bit-7bit
     
    while (NumByteToWrite--)                                     //һ���ֽ�һ���ֽ���ָ����ַд������
    {     
        S25FL132K_SPI_FLASH_SendByte(*pBuffer);           
        pBuffer++;   
    }   
  
    S25FL132K_SPI_FLASH_CS_HIGH();                               //CS����ߵ�ƽ   
    S25FL132K_SPI_FLASH_WaitForWriteEnd();                       //�ȴ�FLASH�Լ�������� 
}   

/**
 *******************************************************************************
 ** \brief �޼���дSPI FLASH 
 **
 ** \param  pBuffer:ָ��Ҫд��flash�Ļ���
 ** \param  WriteAddr:Ҫд��flash�ĵ�ַ
 ** \param  NumByteToWrite:Ҫд��flash����������
 **
 ** \retval ��
 **
 ******************************************************************************/
void S25FL132K_SPI_FLASH_Write_NoCheck(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite)
{
    uint16_t pageremain;
    pageremain = 256 - WriteAddr % 256;            //��ҳʣ����ֽ���
    if (NumByteToWrite <= pageremain)              //������256���ֽ�
    {
        pageremain = NumByteToWrite;
    }
    while (1)
    {
        S25FL132K_SPI_FLASH_PageWrite(pBuffer, WriteAddr, pageremain);
        if (NumByteToWrite == pageremain)
        {
            break;                                //д�������
        }
        else 
        {
            pBuffer        += pageremain;
            WriteAddr      += pageremain;

            NumByteToWrite -= pageremain;         //��ȥ�Ѿ�д���˵��ֽ���
            if (NumByteToWrite > 256)             //һ�ο���д��256���ֽ�
            {
                pageremain = 256;                 //����256���ֽ���
            }
            else
            {
                pageremain = NumByteToWrite;
            }
        }
    }
}

/**
 ******************************************************************************
 ** \brief  ��FLASHָ����ַ��ʼ��ȡָ������������
 **
 ** \param  pBuffer:ָ���Ŷ�ȡ��FLASH������
 ** \param  ReadAddr:Ҫ����������FLASH�е���ʼ��ַ
 ** \param  NumByteToWrite:Ҫ��FLASH��ȡ�����������ֽ���
 ** \return ��
 ******************************************************************************/
void S25FL132K_SPI_FLASH_BufferRead(uint8_t* pBuffer, uint32_t ReadAddr, uint16_t NumByteToRead)   
{     
    S25FL132K_SPI_FLASH_CS_LOW();                                //CS����͵�ƽ
    S25FL132K_SPI_FLASH_SendByte(S25FL132K_ReadData);            //���Ͷ�ȡ���ݵ�ָ��    
    S25FL132K_SPI_FLASH_SendByte((ReadAddr & 0xFF0000) >> 16);   //����Ҫ��ȡ���ݵ�ַ��16bit-23bit    
    S25FL132K_SPI_FLASH_SendByte((ReadAddr & 0xFF00) >> 8);      //����Ҫ��ȡ���ݵ�ַ��8bit-15bit   
    S25FL132K_SPI_FLASH_SendByte((ReadAddr & 0xFF));             //����Ҫ��ȡ���ݵ�ַ��0bit-7bit  
    while (NumByteToRead--)                                      //�����ݵ�ѭ������  
    {     
        *pBuffer = S25FL132K_SPI_FLASH_SendByte(Dummy_Byte);    
        pBuffer++;   
    }   
    S25FL132K_SPI_FLASH_CS_HIGH();                               //CS����ߵ�ƽ
}   

/**
 ******************************************************************************
 ** \brief  д�������ݵ�FLASH��,������д�������������٣�ʹ����ҳд��˳��
 **
 ** \param  pBuffer:ָ��Ҫд��ҳ������ָ��
 ** \param  WriteAddr:Ҫд���ݵ�FLASHĿ���ַ
 ** \param  NumByteToWrite:Ҫд��ҳ�������ֽ���
 ** \return ��
 ******************************************************************************/
uint8_t S25FL132K_BUFFER[4096];
void S25FL132K_SPI_FLASH_BufferWrite(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite)   
{   
    uint32_t secpos;
    uint16_t secoff;
    uint16_t secremain;
    uint16_t i;
    uint8_t * S25FL132K_BUF;
    S25FL132K_BUF = S25FL132K_BUFFER;
    secpos     = WriteAddr / 4096;                 //������ַ 
    secoff     = WriteAddr % 4096;                 //�������ڵ�ƫ��
    secremain  = 4096 - secoff;                    //����ʣ��ռ��С
    if (NumByteToWrite <= secremain)               //С��4096���ֽ�
    {
        secremain = NumByteToWrite;                                
    }
    while (1)
    {
        S25FL132K_SPI_FLASH_BufferRead(S25FL132K_BUF, secpos * 4096, 4096);   //������������������
        for (i = 0; i < secremain; i++)                                    //У������
        {
            if (S25FL132K_BUF[secoff + i] != 0XFF)                            //��Ҫ����
            {
                break;
            }
        }
        if (i < secremain)                                                //��Ҫ����
        {
            S25FL132K_SPI_FLASH_SectorErase(secpos);                      //�����������
            for (i = 0; i < secremain; i++)                               //��������
            {
                S25FL132K_BUF[i + secoff] = pBuffer[i];
            }
            S25FL132K_SPI_FLASH_Write_NoCheck(S25FL132K_BUF, secpos * 4096, 4096); //д���������� 

        }
        else
        {
            S25FL132K_SPI_FLASH_Write_NoCheck(pBuffer, WriteAddr, secremain);   //д�Ѿ������˵�,ֱ��д������ʣ������.
        }
        if (NumByteToWrite == secremain)                                        //д�����
        {
            break;
        }
        else                                                                   //д��δ����
        {
            secpos++;                                                          //������ַ��1
            secoff          = 0;                                               //ƫ��λ��Ϊ0 

            pBuffer        += secremain;                                       //ָ��ƫ��
            WriteAddr      += secremain;                                       //д��ַƫ��
            NumByteToWrite -= secremain;                                       //�ֽ����ݼ�
            if (NumByteToWrite > 4096)                                         //��һ����������д����
            {
                secremain = 4096;
            }
            else                                                               //��һ����������д����
            {
                secremain = NumByteToWrite;
            }
        }
    }
    ;

}   

/**
 ******************************************************************************
 ** \brief  ��ȡFLASH��JEDEC ID
 **
 ** \param  ��
 ** \return FLASH��JEDEC ID
 ******************************************************************************/  
uint32_t S25FL132K_SPI_FLASH_ReadID(void)   
{   
    uint32_t Temp = 0, Temp0 = 0, Temp1 = 0, Temp2 = 0;   
         
    S25FL132K_SPI_FLASH_CS_LOW();                           //CS����͵�ƽ   
    S25FL132K_SPI_FLASH_SendByte(S25FL132K_JedecDeviceID);  //���Ͷ�ȡID��ָ�� 
    Temp0 = S25FL132K_SPI_FLASH_SendByte(Dummy_Byte);       //��ȡ���ֽ�  :Manufacturer 
    Temp1 = S25FL132K_SPI_FLASH_SendByte(Dummy_Byte);       //��ȡ�м��ֽ�:Memory Type  
    Temp2 = S25FL132K_SPI_FLASH_SendByte(Dummy_Byte);       //��ȡ���ֽ�  :Capacity
    S25FL132K_SPI_FLASH_CS_HIGH();                          //CS����ߵ�ƽ       
    Temp = (Temp0 << 16) | (Temp1 << 8) | Temp2;            //������ȡ���ֽ�ƴ��32bit������
    return Temp;   
}   

/**
 ******************************************************************************
** \brief  ��ȡFLASH��Device ID ���͹��̣�ABh->dummy->dummy->dummy
 **
 ** \param  ��
 ** \return FLASH��ID
 ******************************************************************************/    
uint32_t S25FL132K_SPI_FLASH_ReadDeviceID(void)   
{   
    uint32_t Temp = 0;     
    S25FL132K_SPI_FLASH_CS_LOW();      
    S25FL132K_SPI_FLASH_SendByte(S25FL132K_DeviceID);   //����ABh
    S25FL132K_SPI_FLASH_SendByte(Dummy_Byte);           //����dummy
    S25FL132K_SPI_FLASH_SendByte(Dummy_Byte);           //����dummy   
    S25FL132K_SPI_FLASH_SendByte(Dummy_Byte);           //����dummy     
    Temp = S25FL132K_SPI_FLASH_SendByte(Dummy_Byte);    //��ȡDeveice ID
    S25FL132K_SPI_FLASH_CS_HIGH();     
    return Temp;   
}   
  
/**
 ******************************************************************************
 ** \brief  ������ʱ��ָ�����к͵�ַ���еķ���
 **
 ** \param  ReadAddr:�����Ҫ�����ݵ�FLASHĿ���ַ
 ** \return ��
 ******************************************************************************/
void S25FL132K_SPI_FLASH_StartReadSequence(uint32_t ReadAddr)   
{    
    S25FL132K_SPI_FLASH_CS_LOW();                                   //CS����͵�ƽ
    S25FL132K_SPI_FLASH_SendByte(S25FL132K_ReadData);               //���Ͷ�ָ��
    S25FL132K_SPI_FLASH_SendByte((ReadAddr & 0xFF0000) >> 16);      //����Ҫ���ĵ�ַ��bit16-bit23    
    S25FL132K_SPI_FLASH_SendByte((ReadAddr& 0xFF00) >> 8);          //����Ҫ���ĵ�ַ��bit8-bit15    
    S25FL132K_SPI_FLASH_SendByte(ReadAddr & 0xFF);                  //����Ҫ���ĵ�ַ��bit0-bit7
}   
    
/**
 ******************************************************************************
** \brief  ��SPI��һ������
 **
 ** \param  ��
 ** \return ��������ȡ���ֽ�
 ******************************************************************************/
uint8_t S25FL132K_SPI_FLASH_ReadByte(void)   
{   
    return (S25FL132K_SPI_FLASH_SendByte(Dummy_Byte));   
}   
 
/**
 ******************************************************************************
** \brief  ͨ��SPI����һ���ֽ�
 **
 ** \param  ��
 ** \return ����SPI���ݼĴ�������
 ******************************************************************************/
uint8_t S25FL132K_SPI_FLASH_SendByte(uint8_t byte)   
{      
    while(M0P_SPI0->STAT_f.TXE==0);                  //�ȴ����ͻ������ձ�־��λ   
    M0P_SPI0->DATA_f.DAT=byte;                       //�����ݼĴ���д����
    while(M0P_SPI0->STAT_f.RXNE==0);                 //�ȴ����ջ������ǿձ�־��λ
    return M0P_SPI0->DATA_f.DAT;                     //����SPI���߶�ȡ������
}   
   
/**
 ******************************************************************************
** \brief  ����дʹ��ָ�FLASH
 **
 ** \param  ��
 ** \return ��
 ******************************************************************************/  
void S25FL132K_SPI_FLASH_WriteEnable(void)   
{     
    S25FL132K_SPI_FLASH_CS_LOW();                           //CS����͵�ƽ 
    S25FL132K_SPI_FLASH_SendByte(S25FL132K_WriteEnable);    //дָ��
    S25FL132K_SPI_FLASH_CS_HIGH();                          //CS����ߵ�ƽ 
}   
  
/**
 ******************************************************************************
 ** \brief  ��ѯ�ȴ�WIP��־λ����
 **
 ** \param  ��
 ** \return ��
 ******************************************************************************/
void S25FL132K_SPI_FLASH_WaitForWriteEnd(void)   
{   
    uint8_t FLASH_Status = 0;    
    S25FL132K_SPI_FLASH_CS_LOW();                               //CS����͵�ƽ
    S25FL132K_SPI_FLASH_SendByte(S25FL132K_ReadStatusReg);      //���Ͷ�ȡ״̬��־λ��ָ��   
    do   
    {     
        FLASH_Status = S25FL132K_SPI_FLASH_SendByte(Dummy_Byte);   
    }   
    while ((FLASH_Status & WIP_Flag) != 0);                     //�ȴ�״̬��־λ��λ 

    S25FL132K_SPI_FLASH_CS_HIGH();                              //����ߵ�ƽ 
}   
   
/**
 ******************************************************************************
 ** \brief  ʹFLASHоƬ�����Դ�ض�ģʽ
 **
 ** \param  ��
 ** \return ��
 ******************************************************************************/
void S25FL132K_SPI_Flash_PowerDown(void)      
{     
    S25FL132K_SPI_FLASH_CS_LOW();                            //CS����͵�ƽ   
    S25FL132K_SPI_FLASH_SendByte(S25FL132K_PowerDown);       //����Deep-Power-Downָ�� 
    S25FL132K_SPI_FLASH_CS_HIGH();                           //CS����ߵ�ƽ 
}      
   
/**
 ******************************************************************************
 ** \brief  ��Deep-Power-Downģʽ�л���
 **
 ** \param  ��
 ** \return ��
 ******************************************************************************/  
void S25FL132K_SPI_Flash_WAKEUP(void)      
{      
    S25FL132K_SPI_FLASH_CS_LOW();                            //CS����͵�ƽ
    S25FL132K_SPI_FLASH_SendByte(S25FL132K_ReleasePowerDown);//����ABh    
    S25FL132K_SPI_FLASH_CS_HIGH();                           //CS����ߵ�ƽ               
}      
   
/******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
