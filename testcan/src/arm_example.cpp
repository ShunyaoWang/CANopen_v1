#include "ros/ros.h"
#include "testcan/IpPos.h"
#include "iostream"
#include "fstream"
#define postoangle 1440000.0/360 
#define big 2498560.0/360
#define degreetoradius 180.0/3.1415926
#define radiustodegree 3.1415926/180.0

using namespace std;


testcan::IpPos ip_pos;
int sendangle(int id, double angle){
    ip_pos.id = id;
    ip_pos.pos = angle*big*degreetoradius;
    return 0;
}

int main(int argc, char *argv[])
{
    printf("hello\n");
    ros::init(argc, argv, "arm_test_node");
    ros::NodeHandle nh_;
	printf("start");
    ros::Publisher pos_pub = nh_.advertise<testcan::IpPos>("ip_pos",1000);
    /* code for main function */
    // FILE *fp1;
	// fp1 = fopen("catkin_ws/src/testcan/data/joint_straight_line.txt", "r");
    ifstream infile("catkin_ws/src/testcan/data/joint_straight_line.txt");
    // int a;
    double j1,j2,j3,j4,j5,j6,j7;
    infile>>j1>>j2>>j3>>j4>>j5>>j6>>j7;
    cout<<"j1"<<j1<<"j2"<<j2<<"j3"<<j3<<"j4"<<j4<<"j5"<<j5<<"j6"<<j6<<"j7"<<j7;
    // sendangle(1, j1*10000);
    // pos_pub.publish(ip_pos);
    // sendangle(3, j3*10000);
    // pos_pub.publish(ip_pos);
    // sendangle(5, j5*10000);
    // pos_pub.publish(ip_pos);
    sendangle(1, 0);
    pos_pub.publish(ip_pos);
    sendangle(2, 0);
    pos_pub.publish(ip_pos);
    sendangle(3, 0);
    pos_pub.publish(ip_pos);
    sendangle(4, 0);
    pos_pub.publish(ip_pos);
    sendangle(5, 0);
    pos_pub.publish(ip_pos);
    sendangle(6, 0);
    pos_pub.publish(ip_pos);   
    sendangle(7, 0);
    pos_pub.publish(ip_pos);
    usleep(5000000);
    ros::Rate loop_rate(200);
    while (!infile.eof()&&ros::ok())  
	{
    infile>>j1>>j2>>j3>>j4>>j5>>j6>>j7;
    cout<<"j1"<<j1<<"j2"<<j2<<"j3"<<j3<<"j4"<<j4<<"j5"<<j5<<"j6"<<j6<<"j7"<<j7<<endl;
    sendangle(1, j1);
    pos_pub.publish(ip_pos);
    sendangle(2, j2);
    pos_pub.publish(ip_pos);
    sendangle(3, -j3);
    pos_pub.publish(ip_pos);
    sendangle(4, j4);
    pos_pub.publish(ip_pos);
    sendangle(5, -j5);
    pos_pub.publish(ip_pos);
    sendangle(6, j6);
    pos_pub.publish(ip_pos);   
    sendangle(7, -j7);
    pos_pub.publish(ip_pos);

		ros::spinOnce();

		loop_rate.sleep();

	}
    return 0;
}