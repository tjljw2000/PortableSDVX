/* 调试 */
/* 提供printf子程序和延时函数 */

#ifndef	__DEBUG_H__
#define __DEBUG_H__

//定义函数返回值
#ifndef  SUCCESS
#define  SUCCESS  0
#endif
#ifndef  FAIL
#define  FAIL    0xFF
#endif

//定义定时器起始
#ifndef  START
#define  START  1
#endif
#ifndef  STOP
#define  STOP    0
#endif

#define	 FREQ_SYS	     12000000	         //系统主频12MHz
#define  OSC_EN_XT     0                 //外部晶振使能，默认开启内部晶振
#ifndef  UART0_BUAD
#define  UART0_BUAD    57600
#define  UART1_BUAD    57600
#endif
#ifndef  DE_PRINTF                      //程序Printf输出开启
#define  DE_PRINTF     1
#endif

void	CfgFsys( );                        //CH552时钟选择和配置
void	mDelayuS( UINT16 n );              // 以uS为单位延时
void	mDelaymS( UINT16 n );              // 以mS为单位延时
void    CH552UART0Alter();                 //CH552串口0引脚映射到P1.2/P1.3
void	mInitSTDIO( );                      //T1作为波特率发生器
UINT8   CH552UART0RcvByte( );              //CH552 UART0查询方式接收一个字节
void    CH552UART0SendByte(UINT8 SendDat); //CH552 UART0发送一个字节

void    CH552WDTModeSelect(UINT8 mode);     //CH552看门狗模式设置 
void    CH552WDTFeed(UINT8 tim);            //CH552看门狗喂狗

void CH552SoftReset( );                  //CH552软复位
void CH552USBDevWakeup( );               //CH552做USB设备唤醒主机
 
#endif
