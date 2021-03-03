#include "CH552.H"
#include "Debug.H"
#include "USB.H"
#include "Keymap.H"
#include "WS2812.H"

sbit rgb  = P3 ^ 0;    // WS2812B 驱动引脚
sbit key1 = P3 ^ 2;   // 一堆按键(没有用复用, 如果想扩展键盘数量可以考虑用查理 Start
sbit key2 = P1 ^ 4;   // 复用的办法, 7 个管脚可以整 42 个键, -.- 掉头发警告) A
sbit key3 = P1 ^ 5;   // B
sbit key4 = P1 ^ 6;   // C
sbit key5 = P1 ^ 7;   // D 
sbit key6 = P1 ^ 0;   // L
sbit key7 = P1 ^ 1;   // EC11_D / R
sbit EC11_A = P1 ^ 2;
sbit EC11_B = P1 ^ 3; // EC11的A、B脚
sbit EC11_C = P3 ^ 4;
sbit EC11_D = P3 ^ 5; // EC11的C、D脚

main()
{
    CfgFsys();
    mDelaymS(20);
    USBDeviceInit();
    EA = 1;                                                               //允许单片机中断
    UEP1_T_LEN = 0;                                                       //预使用发送长度一定要清空
    UEP2_T_LEN = 0;                                                       //预使用发送长度一定要清空
    FLAG = 0;
    Ready = 0;
    while (1)
    {
        
    }
}