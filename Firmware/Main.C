#include <string.H>
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

UINT8 nkey = 0;
UINT8 result = 0;
UINT8 last_result = 0;

void HIDValueHandleMouse()
{
    TR0 = 0;                     // 发送前关定时器中断
    FLAG = 0;                    // 清空USB中断传输完成标志，准备发送按键按下数据
    Enp1IntIn();                 // USB设备模式端点2的中断上传
    while(FLAG == 0);            // 等待USB中断数据传输完成
    TR0 = 1;                     // 发送完打开定时器中断
}

void keyboardUpKeyValue(UINT8 key)
{
    HIDKey[2] = key;
    while ( (UEP1_CTRL & MASK_UEP_T_RES) == UEP_T_RES_ACK);
    HIDValueHandleMouse();
}

int main()
{
    CfgFsys();
    mDelaymS(5);
    USBDeviceInit();
    
    EA = 1;                                                               //允许单片机中断
    UEP1_T_LEN = 0;                                                       //预使用发送长度一定要清空
    UEP2_T_LEN = 0;                                                       //预使用发送长度一定要清空
    FLAG = 0;
    Ready = 0;

    while (1)
    {
        if (Ready) {
            scan_key();
            if (changed) {
                keyboardUpload();
            }
            keyboardUpKeyValue(keymaps[1]);
        } 
        result = key1;
        mDelaymS(5);
        if (result == key1) {
            if (!result)  nkey = keymaps[1];
            else          nkey = 0;
        }
        if (last_result != result) {
            last_result = result;
            keyboardUpKeyValue(nkey);
        }
			keyboardUpKeyValue(keymaps[0]);
    }
}