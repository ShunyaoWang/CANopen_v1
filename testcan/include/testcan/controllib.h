#ifndef __CONTROLLIB_H
#define __CONTROLLIB_H

#include "can.h"
#include "sdo.h"

//����
#define MOP 0x6060
#define CW 0x6040

#define TP  0x607A 	//Ŀ��λ��
#define TV_V  0x60ff	//���ٶ�ģʽ�µ�Ŀ���ٶ�
#define TV_P  0x6081	//��λ��ģʽ�µ�Ŀ���ٶ�
#define TA  0x6083 //Ŀ����ٶ�
#define TDa 0x6084
#define AP 0x20A0 //Ŀ����ٶ�

//�ŷ�ģʽ
#define SON 0x0f
#define SOFF 0x07
#define SREADY 0x06
#define Sstart 0x1f

#define PPM 0x01
#define PVM 0x03
#define PTM 0x04
#define HM 0x06
#define IPM 0x07



u8 SwitchControlMode(CAN_PORT CANx, u8 id, u8 mode);    //дģʽ
u8 SetPos(CAN_PORT CANx, u8 id, int32_t Pos);                //дĿ��λ��
u8 SetVel_posmod(CAN_PORT CANx, u8 id, int32_t vel);           //��λ��ģʽ��дĿ���ٶ�
u8 SetVel_velmod(CAN_PORT CANx, u8 id, int32_t vel);           //���ٶ�ģʽ��дĿ���ٶ�
u8 SetAcc(CAN_PORT CANx, u8 id, int32_t Acc);           //д���ٶ�
u8 SetDacc(CAN_PORT CANx, u8 id, int32_t Dacc);         //д���ٶ�
//u8 SetTOR(CAN_PORT CANx, u8 id);                     
u8 ServerStatus(CAN_PORT CANx, u8 id, uint16_t mode);  //д������
#endif
