#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <pthread.h>
// #include "controlcan.h"
#include "canopen_vci_ros.h"
// #include "ros/ros.h"
// #include "testcan/Frame.h"
#include "std_msgs/String.h"

#include <ctime>
#include <cstdlib>
#include "unistd.h"
// #define dev_id 0x07
 
// VCI_BOARD_INFO pInfo;//用来获取设备信息。
int count=1,sendcount = 1,packed_num=0;//数据列表中，用来存储列表序号。
testcan::Frame _can_receive;
int CANopenVCIROS::motor_init_pos[28];
VCI_CAN_OBJ psend[51];
// VCI_CAN_OBJ *psend = send;
// VCI_CAN_OBJ send[48];
// char ip_pos[8] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

// class CANopenVCIROS
// {
// public:
//     CANopenVCIROS();
//     virtual ~CANopenVCIROS();
//     ros::Publisher canrecieve_pub;
//     int send(int id, int CANx, int type, const char *pdata);
//     int PDO_init(int dev_id);
//     static testcan::Frame can_receive;
//     static void *receive_func(void* param); 
//     static void cansendCallback(const testcan::Frame::ConstPtr& canmsg);
//     template <typename sdo>
//         char* SDODataPack(sdo data, unsigned short index, char subindex,char (&sdoptr)[8]);
    
// private:
//     VCI_BOARD_INFO pInfo;//用来获取设备信息
//     ros::NodeHandle nh_;
//     // ros::Publisher canrecieve_pub;
//     ros::Subscriber cansend_sub;
//     /* data */
// };
CANopenVCIROS::CANopenVCIROS(){
    
        printf(">>this is hello !\r\n %d",init_structure_motor_pos[17]);


};

CANopenVCIROS::~CANopenVCIROS(){
        VCI_ResetCAN(VCI_USBCAN2, 0, 0);//复位CAN1通道。
        usleep(100000);//延时100ms。
        VCI_ResetCAN(VCI_USBCAN2, 0, 1);//复位CAN2通道。
        usleep(100000);//延时100ms。
        VCI_CloseDevice(VCI_USBCAN2,0);//关闭设备。
};

void CANopenVCIROS::CanDevInit(){
    // printf(">>this is hello !\r\n");//指示程序已运行
        if(VCI_OpenDevice(VCI_USBCAN2,0,0)==1)//打开设备
        {
            printf(">>open deivce success!\n");//打开设备成功
        }else
        {
            printf(">>open deivce error!\n");
            exit(1);
        }
        if(VCI_ReadBoardInfo(VCI_USBCAN2,0,&pInfo)==1)//读取设备序列号、版本等信息。
        {
                    printf(">>Get VCI_ReadBoardInfo success!\n");
            
            //printf(" %08X", pInfo.hw_Version);printf("\n");
            //printf(" %08X", pInfo.fw_Version);printf("\n");
            //printf(" %08X", pInfo.dr_Version);printf("\n");
            //printf(" %08X", pInfo.in_Version);printf("\n");
            //printf(" %08X", pInfo.irq_Num);printf("\n");
            //printf(" %08X", pInfo.can_Num);printf("\n");
            printf(">>Serial_Num:%c", pInfo.str_Serial_Num[0]);
            printf("%c", pInfo.str_Serial_Num[1]);
            printf("%c", pInfo.str_Serial_Num[2]);
            printf("%c", pInfo.str_Serial_Num[3]);
            printf("%c", pInfo.str_Serial_Num[4]);
            printf("%c", pInfo.str_Serial_Num[5]);
            printf("%c", pInfo.str_Serial_Num[6]);
            printf("%c", pInfo.str_Serial_Num[7]);
            printf("%c", pInfo.str_Serial_Num[8]);
            printf("%c", pInfo.str_Serial_Num[9]);
            printf("%c", pInfo.str_Serial_Num[10]);
            printf("%c", pInfo.str_Serial_Num[11]);
            printf("%c", pInfo.str_Serial_Num[12]);
            printf("%c", pInfo.str_Serial_Num[13]);
            printf("%c", pInfo.str_Serial_Num[14]);
            printf("%c", pInfo.str_Serial_Num[15]);
            printf("%c", pInfo.str_Serial_Num[16]);
            printf("%c", pInfo.str_Serial_Num[17]);
            printf("%c", pInfo.str_Serial_Num[18]);
            printf("%c", pInfo.str_Serial_Num[19]);printf("\n");

            printf(">>hw_Type:%c", pInfo.str_hw_Type[0]);
            printf("%c", pInfo.str_hw_Type[1]);
            printf("%c", pInfo.str_hw_Type[2]);
            printf("%c", pInfo.str_hw_Type[3]);
            printf("%c", pInfo.str_hw_Type[4]);
            printf("%c", pInfo.str_hw_Type[5]);
            printf("%c", pInfo.str_hw_Type[6]);
            printf("%c", pInfo.str_hw_Type[7]);
            printf("%c", pInfo.str_hw_Type[8]);
            printf("%c", pInfo.str_hw_Type[9]);printf("\n");	
        }else
        {
            printf(">>Get VCI_ReadBoardInfo error!\n");
            exit(1);
        }

        //初始化参数，严格参数二次开发函数库说明书。
        VCI_INIT_CONFIG config;
        config.AccCode=0;
        config.AccMask=0xFFFFFFFF;
        config.Filter=1;//接收所有帧
        config.Timing0=0x00;/*波特率125 Kbps  0x03  0x1C*/
        config.Timing1=0x14;
        config.Mode=0;//正常模式		
        
        if(VCI_InitCAN(VCI_USBCAN2,0,0,&config)!=1)
        {
            printf(">>Init CAN1 error\n");
            VCI_CloseDevice(VCI_USBCAN2,0);
        }

        if(VCI_StartCAN(VCI_USBCAN2,0,0)!=1)
        {
            printf(">>Start CAN1 error\n");
            VCI_CloseDevice(VCI_USBCAN2,0);

        }

        if(VCI_InitCAN(VCI_USBCAN2,0,1,&config)!=1)
        {
            printf(">>Init can2 error\n");
            VCI_CloseDevice(VCI_USBCAN2,0);

        }
        if(VCI_StartCAN(VCI_USBCAN2,0,1)!=1)
        {
            printf(">>Start can2 error\n");
            VCI_CloseDevice(VCI_USBCAN2,0);

        }
}

void CANopenVCIROS::rosnode(){
    ros::NodeHandle nh_;
    canrecieve_pub = nh_.advertise<testcan::Frame>("can_recieve", 16);
    cansend_sub = nh_.subscribe("can_send", 1, &CANopenVCIROS::cansendCallback);
    ip_pos_sub = nh_.subscribe("ip_pos",8,&CANopenVCIROS::PosCmdCallback);

};

int CANopenVCIROS::PDO_init(int dev_id){
    /*char IP_Mode_init[51][8] = {{0x40, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00},
                                {0x40, 0x18, 0x10, 0x01, 0x00, 0x00, 0x00, 0x00},
                                {0x2B, 0x0C, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00},
                                {0x2F, 0x0D, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00},
                                {0x23, 0x14, 0x10, 0x00, 0x86, 0x00, 0x00, 0x80},
                                {0x23, 0x14, 0x10, 0x00, 0x86, 0x00, 0x00, 0x00},
                                {0x23, 0x16, 0x10, 0x01, 0x00, 0x00, 0x00, 0x00},
                                {0x2B, 0x17, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00},
                                {0x23, 0x05, 0x10, 0x00, 0x80, 0x00, 0x00, 0x00},//关闭同步发生器
                                {0x23, 0x06, 0x10, 0x00, 0xA0, 0x0F, 0x00, 0x00},//设定同步时间 0x0FA0= 4000us
                                {0x23, 0x00, 0x14, 0x01, dev_id, 0x02, 0x00, 0x80},//RPDO1无效
                                {0x2F, 0x00, 0x14, 0x02, 0x01, 0x00, 0x00, 0x00},//RPDO1通信参数,同步循环,收到一个同步帧更新应用
                                {0x2F, 0x00, 0x16, 0x00, 0x00, 0x00, 0x00, 0x00},//清除RPDO映射对象
                                {0x23, 0x00, 0x16, 0x01, 0x08, 0x00, 0x60, 0x60},//RPDO1映射对象6060(模式选择)
                                {0x2F, 0x00, 0x16, 0x00, 0x01, 0x00, 0x00, 0x00},//RPDO1映射对象一个
                                {0x23, 0x00, 0x14, 0x01, dev_id, 0x02, 0x00, 0x00},//RPDO1有效
                                {0x23, 0x01, 0x14, 0x01, dev_id, 0x03, 0x00, 0x80},//RPDO2无效
                                {0x2F, 0x01, 0x14, 0x02, 0x01, 0x00, 0x00, 0x00},//RPDO2通信参数
                                {0x2F, 0x01, 0x16, 0x00, 0x00, 0x00, 0x00, 0x00},
                                {0x23, 0x01, 0x16, 0x01, 0x10, 0x00, 0x40, 0x60},//RPDO2映射对象6040(控制字)
                                {0x23, 0x01, 0x16, 0x02, 0x20, 0x00, 0x7A, 0x60},//RPDO2映射对象607A(目标位置)
                                {0x2F, 0x01, 0x16, 0x00, 0x02, 0x00, 0x00, 0x00},
                                {0x23, 0x01, 0x14, 0x01, dev_id, 0x03, 0x00, 0x00},//RPOD2有效
                                {0x23, 0x02, 0x14, 0x01, dev_id, 0x04, 0x00, 0x80},//RPDO3无效
                                {0x2F, 0x02, 0x14, 0x02, 0x01, 0x00, 0x00, 0x00},
                                {0x2F, 0x02, 0x16, 0x00, 0x00, 0x00, 0x00, 0x00},
                                {0x23, 0x02, 0x16, 0x01, 0x10, 0x01, 0xC1, 0x60},//RPDO3映射对象60C1(位置低位)
                                {0x23, 0x02, 0x16, 0x02, 0x10, 0x02, 0xC1, 0x60},//RPDO3映射对象60C1(位置高位)
                                {0x2F, 0x02, 0x16, 0x00, 0x02, 0x00, 0x00, 0x00},
                                {0x23, 0x02, 0x14, 0x01, dev_id, 0x04, 0x00, 0x00},
                                {0x23, 0x03, 0x14, 0x01, dev_id, 0x05, 0x00, 0x80},
                                {0x23, 0x00, 0x18, 0x01, 0x80+dev_id, 0x01, 0x00, 0x80},//TPDO1无效(可修改参数)
                                {0x2F, 0x00, 0x18, 0x02, 0x01, 0x00, 0x00, 0x00},//TPDO1通信参数
                                {0x2B, 0x00, 0x18, 0x03, 0x00, 0x00, 0x00, 0x00},//TPDO1禁止时间
                                {0x2B, 0x00, 0x18, 0x05, 0x00, 0x00, 0x00, 0x00},//TPOD1事件计时器
                                {0x2F, 0x00, 0x1A, 0x00, 0x00, 0x00, 0x00, 0x00},
                                {0x23, 0x00, 0x1A, 0x01, 0x10, 0x00, 0x41, 0x60},//TPDO1映射对象6041
                                {0x2F, 0x00, 0x1A, 0x00, 0x01, 0x00, 0x00, 0x00},//TPDO1映射对象个数1
                                {0x23, 0x00, 0x18, 0x01, 0x80+dev_id, 0x01, 0x00, 0x00},//TPDO1有效
                                {0x23, 0x01, 0x18, 0x01, 0x80+dev_id, 0x02, 0x00, 0x80},//TPDO2无效(可修改参数)
                                {0x2F, 0x01, 0x18, 0x02, 0x01, 0x00, 0x00, 0x00},//TPDO2通信参数
                                {0x2B, 0x01, 0x18, 0x03, 0x00, 0x00, 0x00, 0x00},
                                {0x2B, 0x01, 0x18, 0x05, 0x00, 0x00, 0x00, 0x00},
                                {0x2F, 0x01, 0x1A, 0x00, 0x00, 0x00, 0x00, 0x00},//清除TPDO映射对象
                                {0x23, 0x01, 0x1A, 0x01, 0x20, 0x00, 0x6C, 0x60},//TPDO2映射对象606C(速度反馈)
                                {0x23, 0x01, 0x1A, 0x02, 0x20, 0x00, 0x64, 0x60},//TPDO2映射对象6064(位置反馈)
                                {0x2F, 0x01, 0x1A, 0x00, 0x02, 0x00, 0x00, 0x00},
                                {0x23, 0x01, 0x18, 0x01, 0x80+dev_id, 0x02, 0x00, 0x00},
                                {0x23, 0x02, 0x18, 0x01, 0x80+dev_id, 0x03, 0x00, 0x80},
                                {0x23, 0x03, 0x18, 0x01, 0x80+dev_id, 0x04, 0x00, 0x80},
                                {0x23, 0x05, 0x10, 0x00, 0x80, 0x00, 0x00, 0x40}};//打开同步发生器*/
        char IP_Mode_init[48][8] = {{0x40, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00},
                                {0x40, 0x18, 0x10, 0x01, 0x00, 0x00, 0x00, 0x00},
                                {0x2B, 0x0C, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00},
                                {0x2F, 0x0D, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00},
                                {0x23, 0x14, 0x10, 0x00, 0x86, 0x00, 0x00, 0x80},
                                {0x23, 0x14, 0x10, 0x00, 0x86, 0x00, 0x00, 0x00},
                                {0x23, 0x16, 0x10, 0x01, 0x00, 0x00, 0x00, 0x00},
                                {0x2B, 0x17, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00},
                                {0x23, 0x00, 0x14, 0x01, dev_id, 0x02, 0x00, 0x80},//RPDO1无效
                                {0x2F, 0x00, 0x14, 0x02, 0xFF, 0x00, 0x00, 0x00},//RPDO1通信参数
                                {0x2F, 0x00, 0x16, 0x00, 0x00, 0x00, 0x00, 0x00},//清除RPDO映射对象
                                {0x23, 0x00, 0x16, 0x01, 0x08, 0x00, 0x60, 0x60},//RPDO1映射对象6060(模式选择)
                                {0x2F, 0x00, 0x16, 0x00, 0x01, 0x00, 0x00, 0x00},//RPDO1映射对象一个
                                {0x23, 0x00, 0x14, 0x01, dev_id, 0x02, 0x00, 0x00},//RPDO1有效
                                {0x23, 0x01, 0x14, 0x01, dev_id, 0x03, 0x00, 0x80},//RPDO2无效
                                {0x2F, 0x01, 0x14, 0x02, 0xFF, 0x00, 0x00, 0x00},//RPDO2通信参数
                                {0x2F, 0x01, 0x16, 0x00, 0x00, 0x00, 0x00, 0x00},
                                {0x23, 0x01, 0x16, 0x01, 0x10, 0x00, 0x40, 0x60},//RPDO2映射对象6040(控制字)
                                {0x23, 0x01, 0x16, 0x02, 0x20, 0x00, 0x7A, 0x60},//RPDO2映射对象607A(目标位置)
                                {0x2F, 0x01, 0x16, 0x00, 0x02, 0x00, 0x00, 0x00},
                                {0x23, 0x01, 0x14, 0x01, dev_id, 0x03, 0x00, 0x00},//RPOD2有效
                                {0x23, 0x02, 0x14, 0x01, dev_id, 0x04, 0x00, 0x80},//RPDO3无效
                                {0x2F, 0x02, 0x14, 0x02, 0xFF, 0x00, 0x00, 0x00},
                                {0x2F, 0x02, 0x16, 0x00, 0x00, 0x00, 0x00, 0x00},
                                {0x23, 0x02, 0x16, 0x01, 0x10, 0x01, 0xC1, 0x60},//RPDO3映射对象60C1(位置低位)
                                {0x23, 0x02, 0x16, 0x02, 0x10, 0x02, 0xC1, 0x60},//RPDO3映射对象60C1(位置高位)
                                {0x2F, 0x02, 0x16, 0x00, 0x02, 0x00, 0x00, 0x00},
                                {0x23, 0x02, 0x14, 0x01, dev_id, 0x04, 0x00, 0x00},
                                {0x23, 0x03, 0x14, 0x01, dev_id, 0x05, 0x00, 0x80},
                                {0x23, 0x00, 0x18, 0x01, 0x80+dev_id, 0x01, 0x00, 0x80},//TPDO1无效(可修改参数)
                                {0x2F, 0x00, 0x18, 0x02, 0xFF, 0x00, 0x00, 0x00},//TPDO1通信参数
                                {0x2B, 0x00, 0x18, 0x03, 0x28, 0x00, 0x00, 0x00},//TPDO1禁止时间
                                {0x2B, 0x00, 0x18, 0x05, 0x00, 0x00, 0x00, 0x00},//TPOD1事件计时器
                                {0x2F, 0x00, 0x1A, 0x00, 0x00, 0x00, 0x00, 0x00},
                                {0x23, 0x00, 0x1A, 0x01, 0x10, 0x00, 0x41, 0x60},//TPDO1映射对象6041
                                {0x2F, 0x00, 0x1A, 0x00, 0x01, 0x00, 0x00, 0x00},//TPDO1映射对象个数1
                                {0x23, 0x00, 0x18, 0x01, 0x80+dev_id, 0x01, 0x00, 0x00},//TPDO1有效
                                {0x23, 0x01, 0x18, 0x01, 0x80+dev_id, 0x02, 0x00, 0x80},//TPDO2无效(可修改参数)
                                {0x2F, 0x01, 0x18, 0x02, 0xFF, 0x00, 0x00, 0x00},
                                {0x2B, 0x01, 0x18, 0x03, 0x28, 0x00, 0x00, 0x00},
                                {0x2B, 0x01, 0x18, 0x05, 0x00, 0x00, 0x00, 0x00},
                                {0x2F, 0x01, 0x1A, 0x00, 0x00, 0x00, 0x00, 0x00},//清除TPDO映射对象
                                {0x23, 0x01, 0x1A, 0x01, 0x20, 0x00, 0x6C, 0x60},//TPDO2映射对象606C(速度反馈)
                                {0x23, 0x01, 0x1A, 0x02, 0x20, 0x00, 0x64, 0x60},//TPDO2映射对象6064(位置反馈)
                                {0x2F, 0x01, 0x1A, 0x00, 0x02, 0x00, 0x00, 0x00},
                                {0x23, 0x01, 0x18, 0x01, 0x80+dev_id, 0x02, 0x00, 0x00},
                                {0x23, 0x02, 0x18, 0x01, 0x80+dev_id, 0x03, 0x00, 0x80},
                                {0x23, 0x03, 0x18, 0x01, 0x80+dev_id, 0x04, 0x00, 0x80}};

                                
        for (int i = 0;i < 48;i++){
            send(dev_id,CAN1,WriteSDO,IP_Mode_init[i],48);
            usleep(10000);
        // printf("OK once\n");
        }
    char modeop[8] = {0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
// char ask[8] = {0x40,0x7A,0x60,0x00,0x00,0x00,0x00,0x00};
    char TPDO_enable[5][8] = {{0x06,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
                    {0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
                    {0x0F,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
                    {0x1F,0x00,0xFF,0xFF,0xFF,0xFF,0x00,0x00},
                    {0x1F,0x00,0x00,0x00,0xF9,0x15,0x00,0x00}};
    char start[8] = {0x01,dev_id,0x00,0x00,0x00,0x00,0x00,0x00};
    // send(dev_id,CAN1,WriteSDO,ask);
    // usleep(10000);    
    send(0,CAN1,0x000,start,3);
    usleep(10000);
    send(dev_id,CAN1,RPDO1,modeop,3);
    usleep(10000);
    send(dev_id,CAN1,RPDO2,TPDO_enable[4],3);
}

void CANopenVCIROS::cansendCallback(const testcan::Frame::ConstPtr& canmsg){
    // VCI_CAN_OBJ send[1];
	/*send[0].ID = canmsg->id;
	send[0].SendType=canmsg->is_error;
	send[0].RemoteFlag=canmsg->is_rtr;
	send[0].ExternFlag=canmsg->is_extended;
	send[0].DataLen=canmsg->dlc;
	
	int i=0;
	for(i = 0; i < send[0].DataLen; i++)
	{
		send[0].Data[i] = canmsg->data[i];
	}
    if(VCI_Transmit(VCI_USBCAN2, 0, CAN1, send, 1) == 1)
		{
			printf("Index:%04d  ",count);count++;
			printf("CAN1 TX ID:0x%08X",send[0].ID);
			if(send[0].ExternFlag==0) printf(" Standard ");
			if(send[0].ExternFlag==1) printf(" Extend   ");
			if(send[0].RemoteFlag==0) printf(" Data   ");
			if(send[0].RemoteFlag==1) printf(" Remote ");
			printf("DLC:0x%02X",send[0].DataLen);
			printf(" data:0x");

			for(i=0;i<send[0].DataLen;i++)
			{
				printf(" %02X",send[0].Data[i]);
			}

			printf("\n");
			send[0].ID+=1;
		}*/
}

void *CANopenVCIROS::receive_func(void* param)  //接收线程。
{
	int reclen=0;
	VCI_CAN_OBJ rec[3000];//接收缓存，设为3000为佳。
	int i,j;
	
    // testcan::Frame can_receive;
    // ros::Publisher canrecieve_pub;
    
	int *run=(int*)param;//线程启动，退出控制。
    int ind=0;
	
	while((*run)&0x0f)
	{
		if((reclen=VCI_Receive(VCI_USBCAN2,0,ind,rec,3000,100))>0)//调用接收函数，如果有数据，进行数据处理显示。
		{
			for(j=0;j<reclen;j++)
			{
				printf("Index:%04d  ",count);count++;//序号递增
				printf("CAN%d RX ID:0x%08X", ind+1, rec[j].ID);//ID
				if(rec[j].ExternFlag==0) printf(" Standard ");//帧格式：标准帧
				if(rec[j].ExternFlag==1) printf(" Extend   ");//帧格式：扩展帧
				if(rec[j].RemoteFlag==0) printf(" Data   ");//帧类型：数据帧
				if(rec[j].RemoteFlag==1) printf(" Remote ");//帧类型：远程帧
				printf("DLC:0x%02X",rec[j].DataLen);//帧长度
				printf(" data:0x");	//数据
                _can_receive.id = rec[j].ID;
                _can_receive.dlc = rec[j].DataLen;
				for(i = 0; i < rec[j].DataLen; i++)
				{
					printf(" %02X", rec[j].Data[i]);
                    _can_receive.data[i] = rec[j].Data[i];
				}
                if (_can_receive.data[0]==0x43){
                    GetInitPos(_can_receive);
                }
                // canrecieve_pub.publish(can_receive);
				//printf(" TimeStamp:0x%08X",rec[j].TimeStamp);//时间标识。
				printf("\n");
                // printf("%d\n",reclen);
			}
		}
	//  ind=!ind;//变换通道号，以便下次读取另一通道，交替读取。		
	}
	printf("run thread exit\n");//退出接收线程	
	pthread_exit(0);
}

void CANopenVCIROS::GetInitPos(testcan::Frame can_receive){
    int dev_id = can_receive.id - ReadSDO;
    unsigned char data[8];
    for(int i = 0; i < 8; i++)
	{
		data[i] = can_receive.data[i];
	} 
    int init_pos = (data[7]<<24)|(data[6]<<16)|(data[5]<<8)|data[4]; 
    motor_init_pos[dev_id] = init_pos; 
    printf("INIT pos of motor %d is %d\n",dev_id,init_pos);
    // printf("encoder %f",two_encoderatio_s);
}

template <typename sdo>
char* CANopenVCIROS::SDODataPack(sdo data, unsigned short index, char subindex,char (&sdoptr)[8]){
    SDOData sdodata;
    char sdoarray[8];
    int i;
    size_t len = sizeof(data);
    sdodata.cmd = 0x23 + 4 * len - 4;
    sdodata.index = index;
    sdodata.subindex = subindex;
    switch (sdodata.cmd)
    {
        case 0x2f:
            /* 1字节数据 */
            sdodata.data[0] = data;
            sdodata.data[1] = 0x00;
            sdodata.data[2] = 0x00;
            sdodata.data[3] = 0x00;
            break;
        case 0x2b:
            sdodata.data[0] = (data<<8)>>8;
            sdodata.data[1] = data>>8;
            sdodata.data[2] = 0x00;
            sdodata.data[3] = 0x00;
            break;
        case 0x27:
            sdodata.data[0] = (data<<16)>>16;
            sdodata.data[1] = (data<<8)>>16;
            sdodata.data[2] = data>>16;
            sdodata.data[3] = 0x00;
            break;
        case 0x23:
            sdodata.data[0] = (data<<24)>>24;
            sdodata.data[1] = (data<<16)>>24;
            sdodata.data[2] = (data<<8)>>24;
            sdodata.data[3] = data>>24;
            break;
    }
    sdoarray[0] = sdodata.cmd;
	sdoarray[1] = (sdodata.index<<8)>>8;
	sdoarray[2] =  sdodata.index>>8;
	sdoarray[3] = sdodata.subindex;
	for (i = 4 ; i < 8 ; i++) {
		sdoarray[i] =  sdodata.data[i-4];
	}
    for (i = 0 ; i < 8 ; i++) {
		sdoptr[i] =  sdoarray[i];
	}
    // return sdoarray;
    

}

int CANopenVCIROS::send(int id, int CANx, int type, const char *pdata, unsigned int frame_num){
    //需要发送的帧，结构体设置
    SDOData sdodata;
    // VCI_CAN_OBJ psend[48];
    if(packed_num<frame_num){
        printf("pack data,pack num = %d\n",packed_num);
        psend[packed_num].ID = type + id;
        psend[packed_num].SendType=0;
        psend[packed_num].RemoteFlag=0;
        psend[packed_num].ExternFlag=0;
        psend[packed_num].DataLen=8;
	
        int i=0;
        for(i = 0; i < psend[packed_num].DataLen; i++)
        {
            psend[packed_num].Data[i] = pdata[i];
        }
        packed_num++;
    }
    printf("send once, packed num = %d, frame num = %d\n",packed_num, frame_num);
    if(packed_num==frame_num){
        packed_num = 0;
        // printf("send once, packed num = %d\n",packed_num);
    	if(VCI_Transmit(VCI_USBCAN2, 0, CANx, psend, frame_num) == 1)
		{
			printf("Index:%04d  ",count);count++;
			printf("CAN1 TX ID:0x%08X",psend[0].ID);
			if(psend[0].ExternFlag==0) printf(" Standard ");
			if(psend[0].ExternFlag==1) printf(" Extend   ");
			if(psend[0].RemoteFlag==0) printf(" Data   ");
			if(psend[0].RemoteFlag==1) printf(" Remote ");
			printf("DLC:0x%02X",psend[0].DataLen);
			printf(" data:0x");

			for(int i=0;i<psend[0].DataLen;i++)
			{
				printf(" %02X",psend[0].Data[i]);
			}

			printf("\n");
			psend[0].ID+=1;
		}
    }

		// if(VCI_Transmit(VCI_USBCAN2, 0, 1, send, 1) == 1)
		// {
		// 	printf("Index:%04d  ",count);count++;
		// 	printf("CAN2 TX ID:0x%08X", send[0].ID);
		// 	if(send[0].ExternFlag==0) printf(" Standard ");
		// 	if(send[0].ExternFlag==1) printf(" Extend   ");
		// 	if(send[0].RemoteFlag==0) printf(" Data   ");
		// 	if(send[0].RemoteFlag==1) printf(" Remote ");
		// 	printf("DLC:0x%02X",send[0].DataLen);
		// 	printf(" data:0x");			
		// 	for(i = 0; i < send[0].DataLen; i++)
		// 	{
		// 		printf(" %02X", send[0].Data[i]);
		// 	}
		// 	printf("\n");
		// 	send[0].ID+=1;
		// }    
}

// char ip_pos[8] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
void CANopenVCIROS::PosCmdCallback(const testcan::IpPos::ConstPtr &cmd){
    char ip_pos[8] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
    int data = cmd->pos;
    int dev_id = cmd->id;
    switch(dev_id){
        case 22:
        case 21:
        case 25:
        case 23:
        case 26:
        case 27:
        case 20:
        case 24:
            data = data - (int)((motor_init_pos[dev_id] -init_structure_motor_pos[dev_id-1]) *two_encoderatio_s);
            if (data < -720000)
                data = data + 1440000;
            if (data > 720000)
                data = data - 1440000;
            break;
        case 17:
        case 12:
        case 15:
        case 10:
        case 14:
        case 13:
        case 11:
        case 16:
            data = data - (int)((motor_init_pos[dev_id] -init_structure_motor_pos[dev_id-1]) *two_encoderatio_l);
            if (data < -1249280)
                data = data + 2498560;
            if (data > 1249280)
                data = data - 2498560;
            break;
    }
    // data = data - (int)((motor_init_pos[dev_id] -init_structure_motor_pos[dev_id-1]) *two_encoderatio_s);
    printf("init_structure_motor_pos of %d is %d",dev_id,init_structure_motor_pos[dev_id-1]);
    printf("send pos is %d",data);
    ip_pos[0] = (data<<24)>>24;
    ip_pos[1] = (data<<16)>>24;
    ip_pos[2] = (data<<8)>>24;
    ip_pos[3] = data>>24;
    
    send(dev_id,CAN1,RPDO3,ip_pos,1);
    printf("send %d times\n",sendcount);
    sendcount++;
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "CANopen_ROS_node");
    CANopenVCIROS CANopenVCIROS1;
    CANopenVCIROS1.CanDevInit();
    CANopenVCIROS1.rosnode();
    ros::NodeHandle nh;
    // ros::NodeHandle nh;
    // ros::Subscriber cansend_sub = nh.subscribe("can_send", 1, CANopenVCIROS1.cansendCallback);
    // CANopenVCIROS CANopenVCIROS1;
    //启动接收线程
    char sdoAP[8]={0x40,0xA0,0x20,0x00,0x00,0x00,0x00,0x00};
    int m_run0=1;
	pthread_t threadid;
	int ret;
	ret=pthread_create(&threadid,NULL,&CANopenVCIROS1.receive_func,&m_run0);
    // CANopenVCIROS1.PDO_init(0x06);
    CANopenVCIROS1.PDO_init(25);
    CANopenVCIROS1.PDO_init(23);
    CANopenVCIROS1.PDO_init(15);
    CANopenVCIROS1.PDO_init(10);
    CANopenVCIROS1.PDO_init(22);
    CANopenVCIROS1.PDO_init(21);
    CANopenVCIROS1.PDO_init(17);
    CANopenVCIROS1.PDO_init(12);
    CANopenVCIROS1.PDO_init(20);
    CANopenVCIROS1.PDO_init(24);
    CANopenVCIROS1.PDO_init(11);
    CANopenVCIROS1.PDO_init(16);
    CANopenVCIROS1.PDO_init(14);
    CANopenVCIROS1.PDO_init(13); 
    CANopenVCIROS1.PDO_init(27);
    CANopenVCIROS1.PDO_init(26);
    // ros::Subscriber ip_pos_sub = nh.subscribe("ip_pos",1,PosCmdCallback);
    // char sdoAP[8]={0x40,0xA0,0x20,0x00,0x00,0x00,0x00,0x00};
    CANopenVCIROS1.send(25,CAN1,WriteSDO,sdoAP,1);
    CANopenVCIROS1.send(23,CAN1,WriteSDO,sdoAP,1);
    CANopenVCIROS1.send(15,CAN1,WriteSDO,sdoAP,1);
    CANopenVCIROS1.send(10,CAN1,WriteSDO,sdoAP,1);
    CANopenVCIROS1.send(22,CAN1,WriteSDO,sdoAP,1);
    CANopenVCIROS1.send(21,CAN1,WriteSDO,sdoAP,1);
    CANopenVCIROS1.send(17,CAN1,WriteSDO,sdoAP,1);
    CANopenVCIROS1.send(12,CAN1,WriteSDO,sdoAP,1);
    CANopenVCIROS1.send(20,CAN1,WriteSDO,sdoAP,1);
    CANopenVCIROS1.send(24,CAN1,WriteSDO,sdoAP,1);
    CANopenVCIROS1.send(11,CAN1,WriteSDO,sdoAP,1);
    CANopenVCIROS1.send(16,CAN1,WriteSDO,sdoAP,1);
    CANopenVCIROS1.send(27,CAN1,WriteSDO,sdoAP,1);
    CANopenVCIROS1.send(26,CAN1,WriteSDO,sdoAP,1);
    CANopenVCIROS1.send(14,CAN1,WriteSDO,sdoAP,1);
    CANopenVCIROS1.send(13,CAN1,WriteSDO,sdoAP,1);
    // CANopenVCIROS1.SDODataPack(0, AP, 0x00,sdo1);

    // CANopenVCIROS1.send(6,CAN1,WriteSDO,sdoAP);

    /* char a[8]={1,2,3,4,5,6,7,8};
            int vel = 1000;
            char sdo1[8]={0};
            
            // char * sdo1 = CANopenVCIROS1.SDODataPack(1000, TV_V, 0x00);
            // sdo2 = &sdo1;
            CANopenVCIROS1.SDODataPack(1000, TV_V, 0x00,sdo1);
            int sdo_num = 48;
            int dev_id = 7;
            char IP_Mode_init[48][8] = {{0x40, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00},
                                        {0x40, 0x18, 0x10, 0x01, 0x00, 0x00, 0x00, 0x00},
                                        {0x2B, 0x0C, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00},
                                        {0x2F, 0x0D, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00},
                                        {0x23, 0x14, 0x10, 0x00, 0x86, 0x00, 0x00, 0x80},
                                        {0x23, 0x14, 0x10, 0x00, 0x86, 0x00, 0x00, 0x00},
                                        {0x23, 0x16, 0x10, 0x01, 0x00, 0x00, 0x00, 0x00},
                                        {0x2B, 0x17, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00},
                                        {0x23, 0x00, 0x14, 0x01, dev_id, 0x02, 0x00, 0x80},
                                        {0x2F, 0x00, 0x14, 0x02, 0xFF, 0x00, 0x00, 0x00},
                                        {0x2F, 0x00, 0x16, 0x00, 0x00, 0x00, 0x00, 0x00},
                                        {0x23, 0x00, 0x16, 0x01, 0x08, 0x00, 0x60, 0x60},
                                        {0x2F, 0x00, 0x16, 0x00, 0x01, 0x00, 0x00, 0x00},
                                        {0x23, 0x00, 0x14, 0x01, dev_id, 0x02, 0x00, 0x00},
                                        {0x23, 0x01, 0x14, 0x01, dev_id, 0x03, 0x00, 0x80},
                                        {0x2F, 0x01, 0x14, 0x02, 0xFF, 0x00, 0x00, 0x00},
                                        {0x2F, 0x01, 0x16, 0x00, 0x00, 0x00, 0x00, 0x00},
                                        {0x23, 0x01, 0x16, 0x01, 0x10, 0x00, 0x40, 0x60},
                                        {0x23, 0x01, 0x16, 0x02, 0x20, 0x00, 0x7A, 0x60},
                                        {0x2F, 0x01, 0x16, 0x00, 0x02, 0x00, 0x00, 0x00},
                                        {0x23, 0x01, 0x14, 0x01, dev_id, 0x03, 0x00, 0x00},
                                        {0x23, 0x02, 0x14, 0x01, dev_id, 0x04, 0x00, 0x80},
                                        {0x2F, 0x02, 0x14, 0x02, 0xFF, 0x00, 0x00, 0x00},
                                        {0x2F, 0x02, 0x16, 0x00, 0x00, 0x00, 0x00, 0x00},
                                        {0x23, 0x02, 0x16, 0x01, 0x10, 0x01, 0xC1, 0x60},
                                        {0x23, 0x02, 0x16, 0x02, 0x10, 0x02, 0xC1, 0x60},
                                        {0x2F, 0x02, 0x16, 0x00, 0x02, 0x00, 0x00, 0x00},
                                        {0x23, 0x02, 0x14, 0x01, dev_id, 0x04, 0x00, 0x00},
                                        {0x23, 0x03, 0x14, 0x01, dev_id, 0x05, 0x00, 0x80},
                                        {0x23, 0x00, 0x18, 0x01, 0x80+dev_id, 0x01, 0x00, 0x80},
                                        {0x2F, 0x00, 0x18, 0x02, 0xFF, 0x00, 0x00, 0x00},
                                        {0x2B, 0x00, 0x18, 0x03, 0x00, 0x00, 0x00, 0x00},
                                        {0x2B, 0x00, 0x18, 0x05, 0x00, 0x00, 0x00, 0x00},
                                        {0x2F, 0x00, 0x1A, 0x00, 0x00, 0x00, 0x00, 0x00},
                                        {0x23, 0x00, 0x1A, 0x01, 0x10, 0x00, 0x41, 0x60},
                                        {0x2F, 0x00, 0x1A, 0x00, 0x01, 0x00, 0x00, 0x00},
                                        {0x23, 0x00, 0x18, 0x01, 0x80+dev_id, 0x01, 0x00, 0x00},
                                        {0x23, 0x01, 0x18, 0x01, 0x80+dev_id, 0x02, 0x00, 0x80},
                                        {0x2F, 0x01, 0x18, 0x02, 0xFF, 0x00, 0x00, 0x00},
                                        {0x2B, 0x01, 0x18, 0x03, 0x00, 0x00, 0x00, 0x00},
                                        {0x2B, 0x01, 0x18, 0x05, 0x00, 0x00, 0x00, 0x00},
                                        {0x2F, 0x01, 0x1A, 0x00, 0x00, 0x00, 0x00, 0x00},
                                        {0x23, 0x01, 0x1A, 0x01, 0x20, 0x00, 0x6C, 0x60},
                                        {0x23, 0x01, 0x1A, 0x02, 0x20, 0x00, 0x64, 0x60},
                                        {0x2F, 0x01, 0x1A, 0x00, 0x02, 0x00, 0x00, 0x00},
                                        {0x23, 0x01, 0x18, 0x01, 0x80+dev_id, 0x02, 0x00, 0x00},
                                        {0x23, 0x02, 0x18, 0x01, 0x80+dev_id, 0x03, 0x00, 0x80},
                                        {0x23, 0x03, 0x18, 0x01, 0x80+dev_id, 0x04, 0x00, 0x80}};

            char test_sdo_pp[6][8] = {{0x2F, 0x60, 0x60, 0x00, 0x01, 0x00, 0x00, 0x00},
                                        {0x23, 0x7A, 0x60, 0x00, 0x00, 0x20, 0x26, 0x00},
                                        {0x23, 0x83, 0x60, 0x00, 0x10, 0x27, 0x00, 0x00},
                                        {0x23, 0x84, 0x60, 0x00, 0x10, 0x27, 0x00, 0x00},
                                        {0x23, 0x81, 0x60, 0x00, 0x10, 0x27, 0x00, 0x00},
                                        {0x2B, 0x40, 0x60, 0x00, 0x1F, 0x00, 0x00, 0x00}};
            // for (int j = 0;j < 6;j++){
            //     CANopenVCIROS1.send(25,CAN1,WriteSDO,test_sdo_pp[j]);
            //     usleep(100000);
            // }
        //     CANopenVCIROS1.send(0,CAN1,0x000,IP_Mode_init[0]);
        //     usleep(10000);
        //     CANopenVCIROS1.send(0,CAN1,0x000,IP_Mode_init[1]);
        //     usleep(10000);
        //     CANopenVCIROS1.send(0,CAN1,0x000,IP_Mode_init[2]);
        //     usleep(10000);
        CANopenVCIROS1.PDO_init(0x07);
            // for (int i = 0;i < sdo_num;i++){
            //     CANopenVCIROS1.send(dev_id,CAN1,WriteSDO,IP_Mode_init[i]);
            //     usleep(10000);
            //     // printf("OK once\n");
            // }
            usleep(100000);

        //     // CANopenVCIROS1.send(0,CAN1,WriteSDO,sdo1);
        //     usleep(1000000);
        // char modeop[8] = {0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
        // char ask[8] = {0x40,0x7A,0x60,0x00,0x00,0x00,0x00,0x00};
        // char TPDO_enable[5][8] = {{0x06,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
        //                     {0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
        //                     {0x0F,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
        //                     {0x1F,0x00,0xFF,0xFF,0xFF,0xFF,0x00,0x00},
        //                     {0x1F,0x00,0x00,0x00,0xF9,0x15,0x00,0x00}};
            char ip_dis[2][8] = {{0x00,0xF9,0x15,0x00,0x00,0x00,0x00,0x00},
                                {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}};
            // char start[8] = {0x01,dev_id,0x00,0x00,0x00,0x00,0x00,0x00};
            //     CANopenVCIROS1.send(dev_id,CAN1,WriteSDO,ask);
            //     usleep(10000);    
            //     CANopenVCIROS1.send(0,CAN1,0x000,start);
            //     usleep(10000);
            //     CANopenVCIROS1.send(dev_id,CAN1,RPDO1,modeop);
            //     usleep(10000);
            //     CANopenVCIROS1.send(dev_id,CAN1,RPDO2,TPDO_enable[4]);
            //     // for (int j = 0;j < 4;j++){
            //     //     CANopenVCIROS1.send(dev_id,CAN1,RPDO2,TPDO_enable[j]);
            //     //     usleep(100000);
            //     // }
            CANopenVCIROS1.send(dev_id,CAN1,RPDO3,ip_dis[0]);
            // usleep(1000000);
            // CANopenVCIROS1.send(dev_id,CAN1,RPDO3,ip_dis[1]);
            // usleep(10000);    
            //CANopenVCIROS1.send(dev_id,CAN1,WriteSDO,ask);*/

    
    
    ros::Rate loop_rate(100);
    usleep(1000000);
    // m_run0=0;//线程关闭指令。
	// pthread_join(threadid,NULL);//等待线程关闭。
 	while (ros::ok())
	{
		CANopenVCIROS1.canrecieve_pub.publish(_can_receive);
        //  ROS_INFO("%s", "ok");
		ros::spinOnce();
        // CANopenVCIROS1.send(0,CAN1,WriteSDO,sdo1);
		loop_rate.sleep();
		// ++count;
	}

    //  usleep(10000000);//延时单位us，这里设置 10 000 000=10s    10s后关闭接收线程，并退出主程序。
	m_run0=0;//线程关闭指令。
	pthread_join(threadid,NULL);//等待线程关闭。

    return 0;
}
