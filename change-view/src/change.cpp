#include<ros/ros.h>
#include<tf/tf.h>
#include<stdlib.h>
#include<view_controller_msgs/CameraPlacement.h>
#include<geometry_msgs/Point.h>
#include<nav_msgs/Odometry.h>

#include <eigen3/Eigen/Geometry>//用到Eigen的几何模块时需要包含的头文件,即变换矩阵的定义等
#include <eigen3/Eigen/Dense>//用到一些常用的函数时，需要包含的头文件
#include <eigen3/Eigen/Eigen>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>


using namespace std;

geometry_msgs::PoseStamped vision;

view_controller_msgs::CameraPlacement camera;

int i=0;

Eigen::Vector3d quaternion_to_euler(const Eigen::Quaterniond &q)
{
    float quat[4];
    quat[0] = q.w();
    quat[1] = q.x();
    quat[2] = q.y();
    quat[3] = q.z();

    Eigen::Vector3d ans;
    ans[0] = atan2(2.0 * (quat[3] * quat[2] + quat[0] * quat[1]), 1.0 - 2.0 * (quat[1] * quat[1] + quat[2] * quat[2]));
    ans[1] = asin(2.0 * (quat[2] * quat[0] - quat[3] * quat[1]));
    ans[2] = atan2(2.0 * (quat[3] * quat[0] + quat[1] * quat[2]), 1.0 - 2.0 * (quat[2] * quat[2] + quat[3] * quat[3]));
    return ans;
}

void vision_odomCallback(const nav_msgs::Odometry::ConstPtr& msg) 
{

	vision.pose.position.x = msg->pose.pose.position.x;
	vision.pose.position.y = msg->pose.pose.position.y;
	vision.pose.position.z = msg->pose.pose.position.z;

	vision.pose.orientation.x = msg->pose.pose.orientation.x;
	vision.pose.orientation.y = msg->pose.pose.orientation.y;
	vision.pose.orientation.z = msg->pose.pose.orientation.z;
	vision.pose.orientation.w = msg->pose.pose.orientation.w;

	Eigen::Quaterniond q1(vision.pose.orientation.w,vision.pose.orientation.x,vision.pose.orientation.y,vision.pose.orientation.z);//定义旋转四元数
    	Eigen::Vector3d eulerAngle1=quaternion_to_euler(q1);
	geometry_msgs::Point pt;

//she zhi yan jing de wei zhi(jiu shi zai na guan cha )

	pt.x=vision.pose.position.x-8.0*cos(eulerAngle1(2));
	pt.y=vision.pose.position.y-8.0*sin(eulerAngle1(2));
	pt.z=vision.pose.position.z;
	if(pt.z<1.0) pt.z=2;

	camera.eye.point=pt;
	camera.eye.header.frame_id = "base_link";
//shi zhe guan zhu jiao dian(ji yao guan cha de wu ti)
	pt.x=vision.pose.position.x;
	pt.y=vision.pose.position.y;
	pt.z=vision.pose.position.z;
	camera.focus.point=pt;
	camera.focus.header.frame_id = "base_link";
//zuo yong hai wei zhi
	geometry_msgs::Point up;
	up.x=0;up.y=0;up.z=0;
	//camera.up.vector = up;
	camera.up.header.frame_id = "base_link";
	camera.time_from_start = ros::Duration();
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "change_view");
    ros::NodeHandle n;

    ros::Subscriber vision_sub = n.subscribe("/vins_estimator/odometry", 10, vision_odomCallback, ros::TransportHints().tcpNoDelay());
    ros::Publisher rviz_change_pub = n.advertise<view_controller_msgs::CameraPlacement>("/rviz/camera_placement",10);

    ros::Rate rate(2);//50-100hz即可


	geometry_msgs::Point pt1;

//she zhi yan jing de wei zhi(jiu shi zai na guan cha )

	pt1.x=-10.0;
	pt1.y=0.0;
	pt1.z=0.5;
	
	camera.eye.point=pt1;
	camera.eye.header.frame_id = "base_link";
//shi zhe guan zhu jiao dian(ji yao guan cha de wu ti)
	pt1.x=0.0;
	pt1.y=0.0;
	pt1.z=0.5;
	camera.focus.point=pt1;
	camera.focus.header.frame_id = "base_link";
//zuo yong hai wei zhi
	geometry_msgs::Vector3 up;
	up.x=0;up.y=0;up.z=1;
	camera.up.vector = up;
	camera.up.header.frame_id = "base_link";
	camera.time_from_start = ros::Duration();
	rviz_change_pub.publish(camera);

	while(ros::ok())
	{
	ros::spinOnce();
	rviz_change_pub.publish(camera);
	}
   	 return 0;
}
