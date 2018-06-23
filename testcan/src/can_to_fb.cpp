#include "testcan/Frame.h"
#include "testcan/canopen_vci_ros.h"
#include "ros/ros.h"
#include "testcan/IpPos.h"
// #include "boost.h"

// int argc_ = 0;
// char *argv_[1] = {0};
// ros::init(argc_, argv_, "cmd_to_can");

// CANopenVCIROS CANopenVCIROS1;
// char ip_pos[8] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
testcan::IpPos cur_pos_msg;

void PosFeedbackCallback(const testcan::Frame::ConstPtr &can_msg){
    unsigned char data[8];
    
    for(int i = 0; i < 8; i++)
	{
		data[i] = can_msg->data[i];
	} 
    int dev_id = can_msg->id - TPDO2;
    // ip_pos[0] = (data<<24)>>24;
    // ip_pos[1] = (data<<16)>>24;
    // ip_pos[2] = (data<<8)>>24;
    // ip_pos[3] = data>>24;

    int cur_pos = (data[7]<<24)|(data[6]<<16)|(data[5]<<8)|data[4];    
    // CANopenVCIROS1.send(0x07,CAN1,RPDO3,ip_pos);
    printf("recieve once, pos is %d\n",cur_pos);//0XFFFFF900);
    cur_pos_msg.pos = cur_pos;
    
    
    float angle = 360*((cur_pos%1440000)/1440000.0);
    if(angle<0){
        angle = angle +360;
    }
    // if(angle>180);angle = 360 - angle;
    // if(angle<-180);angle = 360 + angle;
    cur_pos_msg.angle = angle;
    cur_pos_msg.id = dev_id;
}

int main(int argc, char *argv[])
{
    /* code for main function */
    ros::init(argc, argv, "can_to_fb");
    ros::NodeHandle nh_;
    // CANopenVCIROS CANopenVCIROS1;
    // int m_run0=1;
	// pthread_t threadid;
	// int ret;
	// ret=pthread_create(&threadid,NULL,&CANopenVCIROS1.receive_func,&m_run0);
    // // int a = 0;
    ros::Subscriber ip_pos_sub = nh_.subscribe("can_recieve",1,PosFeedbackCallback);
    ros::Publisher cur_pos_pub = nh_.advertise<testcan::IpPos>("cur_pos",1);
    // testcan::IpPos cur_pos_msg;
    ros::Rate loop_rate(50);
    while (ros::ok)
    {
        /* code for loop body */
        cur_pos_pub.publish(cur_pos_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    // canrecieve_pub = nh_.advertise<testcan::Frame>("can_recieve", 1);
    // CANopenVCIROS CANopenVCIROS1;
    // int dev_id = 0x07;
    // CANopenVCIROS1.PDO_init(0x07);
    // char ip_dis[2][8] = {{0x00,0xF9,0x15,0x00,0x00,0x00,0x00,0x00},
    //                 {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}};
    // CANopenVCIROS1.send(dev_id,CAN1,RPDO3,ip_dis[0]);
    // usleep(1000000);
    // CANopenVCIROS1.send(dev_id,CAN1,RPDO3,ip_dis[1]);
    // ros::spin();
    // printf("once");
    return 0;
}