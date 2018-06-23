#ifndef __CANOPEN_VCI_ROS_H
#define __CANOPEN_VCI_ROS_H

#include "controlcan.h"
// #include "canopen_vci_ros.h"
#include "ros/ros.h"
#include "testcan/Frame.h"
#include "testcan/IpPos.h"

#define MOP 0x6060
#define CW 0x6040

#define TP  0x607A 	//Ŀ��λ��
#define TV_V  0x60ff	//���ٶ�ģʽ�µ�Ŀ���ٶ�
#define TV_P  0x6081	//��λ��ģʽ�µ�Ŀ���ٶ�
#define TA  0x6083 //Ŀ����ٶ�
#define TDa 0x6084 //Ŀ����ٶ�

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

#define two_encoderatio_s 1440000.0/262144.0
#define two_encoderatio_l 2498560.0/262144.0

class CANopenVCIROS
{
public:
    CANopenVCIROS();
    virtual ~CANopenVCIROS();
    ros::Publisher canrecieve_pub;
    static int send(int id, int CANx, int type, const char *pdata,unsigned int frame_num);
    int PDO_init(int dev_id);
    void CanDevInit();
    void rosnode();
    static testcan::Frame can_receive;
    static int motor_init_pos[28];
    // static VCI_CAN_OBJ psend[48];
    // static VCI_CAN_OBJ *psend = send;
    static int init_structure_motor_pos[28];
    static void GetInitPos(testcan::Frame can_receive);
    static void *receive_func(void* param); 
    static void cansendCallback(const testcan::Frame::ConstPtr& canmsg);
    static void PosCmdCallback(const testcan::IpPos::ConstPtr& cmd);
    template <typename sdo>
        char* SDODataPack(sdo data, unsigned short index, char subindex,char (&sdoptr)[8]);    
private:
    VCI_BOARD_INFO pInfo;//用来获取设备信息
    // static VCI_CAN_OBJ send[48];
    // static VCI_CAN_OBJ *psend = send;
    // ros::NodeHandle nh_;
    // ros::Publisher canrecieve_pub;
    // static int motor_init_pos[16];
    ros::Subscriber cansend_sub, ip_pos_sub;
    /* data */
};
// int CANopenVCIROS::init_structure_motor_pos[28];
int CANopenVCIROS::init_structure_motor_pos[28] = {0,
0,
0,
0,
0,
0,
0,
0,
0,
202326,
106257,
33616,
249360,
171466,
195997,
164837,
259432,
0,
0,
34049,
175063,
120943,
193599,
198955,
252825,
235186,
197955,
0};



typedef enum{
    WriteSDO = 0x600,
    ReadSDO = 0x580,
    TPDO1 = 0x180,
    TPDO2 = 0x280,
    RPDO1 = 0x200,
    RPDO2 = 0x300,
    RPDO3 = 0x400
}CanSendType;

typedef enum{
    CAN1 = 0,
    CAN2 = 1
}CANx;

typedef struct {
  char cmd;	/**< sdo's command */
  short index;		/**< index */
  char subindex;		/**<subindex */
  char data[4]; /**< sdo's datas */
} SDOData;

#endif
