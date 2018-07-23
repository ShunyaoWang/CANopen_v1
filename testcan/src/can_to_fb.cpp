#include "testcan/Frame.h"
#include "testcan/canopen_vci_ros.h"
#include "ros/ros.h"
#include "testcan/IpPos.h"
#include "geometry_msgs/Twist.h"
// #include "boost.h"

// int argc_ = 0;
// char *argv_[1] = {0};
// ros::init(argc_, argv_, "cmd_to_can");

// CANopenVCIROS CANopenVCIROS1;
// char ip_pos[8] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
testcan::IpPos cur_pos_msg, absolute_pos_msg, vel_msg,Ic_msg, ip_pos_msg;
int last_motor_pos, last_out_pos, motor_pos, out_pos;
int flag = 0;
FILE *fp1;
// fp1 = fopen("catkin_ws/src/testcan/log/IandVel.txt", "w");
void PosFeedbackCallback(const testcan::Frame::ConstPtr &can_msg){
    unsigned char data[8];
    int dev_id;
    int Iom;
    Iom = 0;
    int vel;
    for(int i = 0; i < 8; i++)
	{
		data[i] = can_msg->data[i];
	} 
    // if (can_msg->id<ReadSDO){
    //     dev_id = can_msg->id - TPDO2;
    // }else{
    //     dev_id = can_msg->id - ReadSDO;
    // }
    // ip_pos[0] = (data<<24)>>24;
    // ip_pos[1] = (data<<16)>>24;
    // ip_pos[2] = (data<<8)>>24;
    // ip_pos[3] = data>>24;

    int cur_pos = (data[7]<<24)|(data[6]<<16)|(data[5]<<8)|data[4]; 
    int16_t I_raw = (data[5]<<8)|data[4];
    if (I_raw<=1000){
        if (I_raw>=-1000){
            Iom=I_raw;
        }
        // I=I_raw;
    }
    
    vel = (data[3]<<24)|(data[2]<<16)|(data[1]<<8)|data[0];
    // CANopenVCIROS1.send(0x07,CAN1,RPDO3,ip_pos);
    // printf("recieve once, pos is %d\n",cur_pos);//0XFFFFF900);
    // cur_pos_msg.pos = cur_pos;
    
    
    float angle = 360*((cur_pos%1440000)/1440000.0);
    if(angle<0){
        angle = angle +360;
    }

    if (can_msg->id<ReadSDO){
        dev_id = can_msg->id - TPDO2;
        cur_pos_msg.pos = cur_pos;
        cur_pos_msg.angle = angle;
        cur_pos_msg.id = dev_id;
        Ic_msg.id = dev_id;
        Ic_msg.pos = Iom;
        vel_msg.id = dev_id;
        vel_msg.angle = 60.0*vel/15616.0/160;
    }else{
        dev_id = can_msg->id - ReadSDO;
        // printf("recieve once, type is %d\n",data[1]);
        if (data[1]==0x64){
            cur_pos_msg.pos = cur_pos%15616;
            cur_pos_msg.angle = angle;
            cur_pos_msg.id = dev_id;
            printf("recieve once, cur pos is %d\n",cur_pos%15616);
            motor_pos = cur_pos;
        }
        if (data[1]==0xA0){
            // if (cur_pos<=262144){
                absolute_pos_msg.pos = cur_pos%262144;
                absolute_pos_msg.angle = 360*(cur_pos%262144)/262144;
                absolute_pos_msg.id = dev_id;  
                printf("recieve once, absolute pos is %d\n",cur_pos%262144);  
            // }   
                out_pos =cur_pos;
        }
        if (data[1]==0x78){//电流
            if (Iom<=1000){
                Ic_msg.pos = Iom;
                Ic_msg.angle = angle;
                Ic_msg.id = dev_id;  
                printf("recieve once, Ic is %d\n",Iom);  
                
            }        
        }
        if (data[1]==0x69){//速度
            // if (cur_pos<=262144){
                vel_msg.pos = cur_pos;
                vel_msg.angle = 60.0*cur_pos/15616.0;;
                vel_msg.id = dev_id;  
                // printf("recieve once, output vel is %f rpm\n",vel_msg.angle);  
            // }        
        }
        // absolute_pos_msg.pos = cur_pos;
        // absolute_pos_msg.angle = angle;
        // absolute_pos_msg.id = dev_id;
    }
    // if (last_out_pos == 0){
    //     last_motor_pos = motor_pos;
    //     last_out_pos = out_pos;
    //     // flag = 1;
    // }
    // if (last_out_pos!=0){
    // int pos_diff = out_pos - last_out_pos;
    // int motor_diff = motor_pos - last_motor_pos;
    // int addpos = pos_diff * two_encoderatio_l - motor_diff;
    // ip_pos_msg.id = 3;
    // if (abs(pos_diff)>1){
    //     ip_pos_msg.pos =out_pos + addpos*100;
    // }
    // printf("pos diff is %d\n add pos is %d/n",pos_diff,addpos);
    // }
    // // last_motor_pos = motor_pos;
    // // last_out_pos = out_pos;
    // // if(angle>180);angle = 360 - angle;
    // // if(angle<-180);angle = 360 + angle;
    // // cur_pos_msg.angle = angle;
    // // cur_pos_msg.id = dev_id;
    // fprintf(fp1, "%d %f\n",Ic_msg.pos, vel_msg.angle);
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
    fp1 = fopen("catkin_ws/src/testcan/log/IandVel.txt", "w");
    fprintf(fp1, "%s %s\n","I", "V");
    ros::Subscriber ip_pos_sub = nh_.subscribe("can_recieve",32,PosFeedbackCallback);
    ros::Publisher cur_pos_pub = nh_.advertise<testcan::IpPos>("cur_pos",1);
    ros::Publisher absolute_pos_pub = nh_.advertise<testcan::IpPos>("absolute_pos",1);
    ros::Publisher vel_pub = nh_.advertise<testcan::IpPos>("velocity",1);
    ros::Publisher ic_pub = nh_.advertise<testcan::IpPos>("I_c",1);
    ros::Publisher pos_pub = nh_.advertise<testcan::IpPos>("ip_pos",1);
    // testcan::IpPos cur_pos_msg;
    ros::Rate loop_rate(200);
    while (ros::ok)
    {
        /* code for loop body */
        cur_pos_pub.publish(cur_pos_msg);
        absolute_pos_pub.publish(absolute_pos_msg);
        vel_pub.publish(vel_msg);
        ic_pub.publish(Ic_msg);
        // pos_pub.publish(ip_pos_msg);
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