#ifndef __USB_H_
#define __USB_H_

extern UINT8X  Ep0Buffer[];
extern UINT8X  Ep1Buffer[];
extern UINT8X  Ep2Buffer[];
extern UINT8   SetupReq, SetupLen, Ready, Count, FLAG, UsbConfig;
//PUINT8  pDescr;                                                                //USB配置标志
extern USB_SETUP_REQ   SetupReqBuf; 

//#ifndef	__DEBUG_H__
//void CH552SoftReset();
//void CH552USBDevWakeup();
//#endif

void USBDeviceInit();
void Enp1IntIn();
void Enp2IntIn();

void HIDValueHandle();
void HIDValueHandleMul();
void HIDValueHandleMouse();

#endif