#include "stdafx.h"
#include <string>
#include <iostream>
#include <stdio.h>
#include "ros.h"
#include "ros/time.h"
#include <windows.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <kinect_v2/BodyJoints.h>
#include <math.h>
// Kinect for Windows SDK Header
#include <Kinect.h>
#include <chrono>
#include <vector>
//#include <Eigen/Dense>


#define TURN_LEFT 0x00
#define TURN_RIGHT 0x01
#define PUSH 0x02
#define PULL 0x03
#define DATA_OVERFLOW 0x04
#define FINISHED 0x05
#define KINECT_PITCH 45

using namespace std::chrono;
using namespace std;
using namespace Eigen;

double r_x(double x, double y, double z, double angle);
double r_y(double x, double y, double z, double angle);
double pitch_rad = 1.0/180.0 * 3.1415926 * KINECT_PITCH;

const   string  get_name(int n);	//This function is to get the name of joint by jointcount
void	reset_body(kinect_v2::BodyJoints& body);	//This function is to reset the joints value of untracked persons
void	reset_joint(geometry_msgs::Pose& joint);    //This function is to reset the single untracked joint
kinect_v2::BodyJoints& get_body(int n, kinect_v2::BodyJoints& person_0, kinect_v2::BodyJoints& person_1, kinect_v2::BodyJoints& person_2,
	kinect_v2::BodyJoints& person_3, kinect_v2::BodyJoints& person_4, kinect_v2::BodyJoints& person_5);	//This function is to return the body data given its user_id

int main(int argc, _TCHAR* argv[])
{
	Eigen::Matrix3f kinect_pinoneer_extrinsic_rot;
	kinect_pinoneer_extrinsic_rot << 0, sin(pitch_rad), cos(pitch_rad), 1, 0, 0, 0, cos(pitch_rad), -sin(pitch_rad);
	
	cout << kinect_pinoneer_extrinsic_rot << endl;
	// Get ready to read Kinect data
	IKinectSensor* mySensor = nullptr;
	GetDefaultKinectSensor(&mySensor);
	mySensor->Open();

	int myBodyCount = 0;
	IBodyFrameSource* myBodySource = nullptr;
	IBodyFrameReader* myBodyReader = nullptr;
	ICoordinateMapper* mycoodinatemapper = nullptr;
	IDepthFrameSource* myDepthFrameSource = nullptr;
	IDepthFrameReader* myDepthFrameReader = nullptr;
	IDepthFrame* mydepthframe = nullptr;

	geometry_msgs::Point prev_point;
	bool is_first_frame = true;
	clock_t left_timer = clock();
	clock_t CMD_timer = clock();
	milliseconds prev_point_time;
	bool record_data = false;
	vector<geometry_msgs::Point> recorded_points_array;
	mySensor->get_DepthFrameSource(&myDepthFrameSource);
	myDepthFrameSource->OpenReader(&myDepthFrameReader);




	mySensor->get_BodyFrameSource(&myBodySource);
	myBodySource->OpenReader(&myBodyReader);
	myBodySource->get_BodyCount(&myBodyCount);

	IBodyFrame* myBodyFrame = nullptr;

	// Get ready to start ROS
	ros::NodeHandle nh;
	char master_uri[] = "192.168.50.127";
	char* ros_master = master_uri;

	cout << "Connecting to server at\n" << ros_master << endl;
	//init a ROS node with a IP Address
	nh.initNode(ros_master);

	cout << "Advertising six persons' joints message\n" << endl;
	kinect_v2::BodyJoints body_0, body_1, body_2, body_3, body_4, body_5;
	std_msgs::Bool Left, Right, Push, Pull;
	std_msgs::String str;
	ros::Publisher body_0_pub("user_0", &body_0);
	ros::Publisher body_1_pub("user_1", &body_1);
	ros::Publisher body_2_pub("user_2", &body_2);
	ros::Publisher body_3_pub("user_3", &body_3);
	ros::Publisher body_4_pub("user_4", &body_4);
	ros::Publisher body_5_pub("user_5", &body_5);

	nh.advertise(body_0_pub);
	nh.advertise(body_1_pub);
	nh.advertise(body_2_pub);
	nh.advertise(body_3_pub);
	nh.advertise(body_4_pub);
	nh.advertise(body_5_pub);

	ros::Publisher Left_pub("sig_left", &Left);
	nh.advertise(Left_pub);

	int track_id = 0;

	cout << "Go robot go!\n" << endl;
	Left.data = true;


	// Loop start
	while (1)
	{
		// get joints data from Kinect
		//S_OK ==> success code break loop if get data
		while (myBodyReader->AcquireLatestFrame(&myBodyFrame) != S_OK);

		int myBodyCount = 0;
		IBody** bodyArr = nullptr;
		myBodySource->get_BodyCount(&myBodyCount); //Get tracked body number
		bodyArr = new IBody * [myBodyCount];
		for (int i = 0; i < myBodyCount; i++)   //Initialize bodyArr
			bodyArr[i] = nullptr;

		//save body information to myBodyFrame
		myBodyFrame->GetAndRefreshBodyData(myBodyCount, bodyArr);
		for (int i = 0; i < myBodyCount; i++)   //Traverse all 6 person
		{
			get_body(i, body_0, body_1, body_2, body_3, body_4, body_5).user_id = i;
			BOOLEAN     result = false;
			if (bodyArr[i]->get_IsTracked(&result) == S_OK && result)   //Judge whether the person get tracked
			{
				get_body(i, body_0, body_1, body_2, body_3, body_4, body_5).tracked = "YES";
				// cout << "Body " << i << " tracked!" << endl;
				track_id = i;
				int count = 0;
				int tracked_count = 0;

				/*
				typedef struct _Joint
				{
				JointType JointType;
				CameraSpacePoint Position;
				TrackingState TrackingState;
				} 	Joint;

				For Joint Type
					JointType_HandTipLeft	= 21,
					JointType_ThumbLeft	= 22,
					JointType_HandTipRight	= 23,
					JointType_ThumbRight	= 24,

				*/
				Joint   jointArr[JointType_Count];
				JointOrientation	orientationArr[JointType_Count];
				bodyArr[i]->GetJoints(JointType_Count, jointArr);    //Get position info of joints
				bodyArr[i]->GetJointOrientations(JointType_Count, orientationArr);    //Get orientation info of joints
				//Ireration with each Joint, the interested joint is LEFT/RIGHT ELBOW/HAND (5,7/9,11) 

				for (int j = 0; j < JointType_Count; j++)
				{
					//jointArr[j].JointType type is int
					//
					string  joint_name = get_name(jointArr[j].JointType);   //Get the name of joint
					if (joint_name != "NULL")   //output joint info
					{
						if (joint_name == "right_hand") {
							//cout << jointArr[j].Position.X << jointArr[j].Position.Y << jointArr[j].Position.Z <<endl;
							//printf("Left_Hand: X %f,\tY:%f,\tZ:%f\r\n", jointArr[j].Position.X, jointArr[j].Position.Y, jointArr[j].Position.Z);
							float velocity = 0.0;
							if (is_first_frame) {
								prev_point_time = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
								prev_point.x = jointArr[j].Position.X;
								prev_point.y = jointArr[j].Position.Y;
								prev_point.z = jointArr[j].Position.Z;
								is_first_frame = false;
							}
							else {
								milliseconds curr_time = duration_cast<milliseconds>(system_clock::now().time_since_epoch());

								//ros::Time curr_time = ros::Time::now();
								//milliseconds diff = ;
								std::chrono::duration<double, std::micro> diff = curr_time - prev_point_time;
								velocity = sqrt((prev_point.x - jointArr[j].Position.X) * (prev_point.x - jointArr[j].Position.X)
									+ (prev_point.y - jointArr[j].Position.Y) * (prev_point.y - jointArr[j].Position.Y)
									+ (prev_point.z - jointArr[j].Position.Z) * (prev_point.z - jointArr[j].Position.Z)) / diff.count() * 1000000;
								//printf("Body idx: %d\tTime Diff:%f us\tLeft Hand Velocity = %f Time_interval:%f\r\n", i, diff.count(), velocity, ((float)(clock() - left_timer) / CLOCKS_PER_SEC));
								cout << ((float)(clock() - CMD_timer) / CLOCKS_PER_SEC) << endl;
								if (velocity >= 1.5) {
									if (((float)(clock() - left_timer) / CLOCKS_PER_SEC) >= 1) {
										//transform here
										//Trigger record
										//cout << ((float)(clock() - CMD_timer) / CLOCKS_PER_SEC) << endl;
										if (((float)(clock() - CMD_timer) / CLOCKS_PER_SEC) >= 5) {
											record_data = true;
										}
										else {
											cout << "Command sleeping!" << endl;
										}

										//Left.data = true;
										//printf("Left Triggered !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\r\n");
										//reset timer
										left_timer = clock();
									}
									else {
										//cout << "Timer is preparing" << endl;
									}
								}
								prev_point_time = curr_time;
								prev_point.x = jointArr[j].Position.X;
								prev_point.y = jointArr[j].Position.Y;
								prev_point.z = jointArr[j].Position.Z;

								if (record_data) {
									//save data
									if (recorded_points_array.size() <= 20) {
										geometry_msgs::Point curr_Point;
										curr_Point.x = jointArr[j].Position.X;
										curr_Point.y = jointArr[j].Position.Y;
										curr_Point.z = jointArr[j].Position.Z;
										recorded_points_array.push_back(curr_Point);

										double diff_x(0), diff_y(0), diff_z(0);
										diff_x = recorded_points_array[recorded_points_array.size() - 1].x - recorded_points_array[0].x;
										diff_y = recorded_points_array[recorded_points_array.size() - 1].y - recorded_points_array[0].y;
										diff_z = recorded_points_array[recorded_points_array.size() - 1].z - recorded_points_array[0].z;

										if (diff_x > 0.6) {
											printf("CMD: Turn Right\r\n");

											record_data = false;
										}
										else if (diff_x < -0.6) {
											printf("CMD: Turn Left\r\n");

											record_data = false;
										}
										else if (diff_z < -0.3) {
											//Push
											printf("CMD: Push\r\n");

											record_data = false;
										}
										else if (diff_z > 0.3) {
											//Pull
											printf("CMD: PULL\r\n");
											record_data = false;
										}
										if (!record_data) {
											recorded_points_array.clear();
											CMD_timer = clock();
										}
									}
									else {
										printf("No Command Detected\r\n");
										record_data = false;
										recorded_points_array.clear();
										CMD_timer = clock();
									}


								}

							}


						}



						//if (joint_name == "right_hand") {
						//	printf("Right_Hand: X %f,\tY:%f,\tZ:%f\r\n", jointArr[j].Position.X, jointArr[j].Position.Y, jointArr[j].Position.Z);

						//}
						if (jointArr[j].TrackingState == TrackingState_Tracked) //Judge the joints whether get tracked
						{
							++tracked_count;
							get_body(i, body_0, body_1, body_2, body_3, body_4, body_5).joints[count].position.x = jointArr[j].Position.X;
							get_body(i, body_0, body_1, body_2, body_3, body_4, body_5).joints[count].position.y = jointArr[j].Position.Y;
							get_body(i, body_0, body_1, body_2, body_3, body_4, body_5).joints[count].position.z = jointArr[j].Position.Z;

							get_body(i, body_0, body_1, body_2, body_3, body_4, body_5).joints[count].orientation.w = orientationArr[j].Orientation.w;
							get_body(i, body_0, body_1, body_2, body_3, body_4, body_5).joints[count].orientation.x = orientationArr[j].Orientation.x;
							get_body(i, body_0, body_1, body_2, body_3, body_4, body_5).joints[count].orientation.y = orientationArr[j].Orientation.y;
							get_body(i, body_0, body_1, body_2, body_3, body_4, body_5).joints[count].orientation.z = orientationArr[j].Orientation.z;
						}
						else
							reset_joint(get_body(i, body_0, body_1, body_2, body_3, body_4, body_5).joints[count]);
						if (joint_name == "head")	// joint "head" doesn't have orientation feature， so just set it to default orientation
						{
							get_body(i, body_0, body_1, body_2, body_3, body_4, body_5).joints[count].orientation.w = 1;
							get_body(i, body_0, body_1, body_2, body_3, body_4, body_5).joints[count].orientation.x = 0;
							get_body(i, body_0, body_1, body_2, body_3, body_4, body_5).joints[count].orientation.y = 0;
							get_body(i, body_0, body_1, body_2, body_3, body_4, body_5).joints[count].orientation.z = 0;
						}
						++count;
					}
				}
				//cout << tracked_count << " joints tracked for the " << i << "th person." << endl;
			}
			else
				reset_body(get_body(i, body_0, body_1, body_2, body_3, body_4, body_5));
		}
		//Processing our algorithm here for specific joint number

		myBodyFrame->Release();
		delete[] bodyArr;
		/*
		switch (track_id)
		{
		case	0:body_0_pub.publish(&body_0); break;
		case	1:body_1_pub.publish(&body_1); break;
		case	2:body_2_pub.publish(&body_2); break;
		case	3:body_3_pub.publish(&body_3); break;
		case	4:body_4_pub.publish(&body_4); break;
		case	5:body_5_pub.publish(&body_5); break;
		}
		*/

		int selected_body = 0;
		if (body_0.tracked == "YES") {
			selected_body = 0;
		}
		else if (body_1.tracked == "YES") {
			selected_body = 1;
		}
		else if (body_2.tracked == "YES") {
			selected_body = 2;
		}
		else if (body_3.tracked == "YES") {
			selected_body = 3;
		}
		else if (body_4.tracked == "YES") {
			selected_body = 4;
		}
		else if (body_5.tracked == "YES") {
			selected_body = 5;
		}
		
		//Right Hand
		printf("X:%2f, Y:%2f, Z:%2f, W:%2f, X:%2f, Y:%2f, Z:%2f.\r\n",
			get_body(selected_body, body_0, body_1, body_2, body_3, body_4, body_5).joints[8].position.x,
			get_body(selected_body, body_0, body_1, body_2, body_3, body_4, body_5).joints[8].position.y,
			get_body(selected_body, body_0, body_1, body_2, body_3, body_4, body_5).joints[8].position.z,
			get_body(selected_body, body_0, body_1, body_2, body_3, body_4, body_5).joints[8].orientation.w,
			get_body(selected_body, body_0, body_1, body_2, body_3, body_4, body_5).joints[8].orientation.x,
			get_body(selected_body, body_0, body_1, body_2, body_3, body_4, body_5).joints[8].orientation.y,
			get_body(selected_body, body_0, body_1, body_2, body_3, body_4, body_5).joints[8].orientation.z);
		
		//HEAD
		printf("X:%2f, Y:%2f, Z:%2f, W:%2f, X:%2f, Y:%2f, Z:%2f.\r\n",
			get_body(selected_body, body_0, body_1, body_2, body_3, body_4, body_5).joints[0].position.x,
			get_body(selected_body, body_0, body_1, body_2, body_3, body_4, body_5).joints[0].position.y,
			get_body(selected_body, body_0, body_1, body_2, body_3, body_4, body_5).joints[0].position.z,
			get_body(selected_body, body_0, body_1, body_2, body_3, body_4, body_5).joints[0].orientation.w,
			get_body(selected_body, body_0, body_1, body_2, body_3, body_4, body_5).joints[0].orientation.x,
			get_body(selected_body, body_0, body_1, body_2, body_3, body_4, body_5).joints[0].orientation.y,
			get_body(selected_body, body_0, body_1, body_2, body_3, body_4, body_5).joints[0].orientation.z);
		


		//POST A SERIAL DATA HERE
		// 
		// 
		// 
		

		//cout << get_body(selected_body, body_0, body_1, body_2, body_3, body_4, body_5).joints[8].position.x << endl;

		/*body_0_pub.publish(&body_0);
		body_1_pub.publish(&body_1);
		body_2_pub.publish(&body_2);
		body_3_pub.publish(&body_3);
		body_4_pub.publish(&body_4);
		body_5_pub.publish(&body_5)*/;

		Sleep(50);

		nh.spinOnce();
	}
	myBodyReader->Release();
	myBodySource->Release();
	mySensor->Close();
	mySensor->Release();

	printf("All done!\n");
	return 0;
}

//typedef int32_t _user_id_type;
//_user_id_type user_id;
//typedef const char* _tracked_type;
//_tracked_type tracked;
//geometry_msgs::Pose joints[16];

//Pose
//typedef geometry_msgs::Point _position_type;
//_position_type position;
//	double x,y,z
//typedef geometry_msgs::Quaternion _orientation_type;
//_orientation_type orientation;
//	double x,y,z,w


const   string  get_name(int n)		// only remain key 16 joints in consideration of the limitation on message size
{
	//Change Here to increase or delete joints
	switch (n)
	{
	case	0:return	"base_spine"; break;
	case    2:return    "neck"; break;
	case    3:return    "head"; break;

	case    4:return    "left_shoulder"; break;
	case	5:return	"left_elbow"; break; //proces these two parameter elbow and hand
	case    7:return    "left_hand"; break; //

	case    8:return    "right_shoulder"; break;
	case	9:return	"right_elbow"; break;
	case    11:return   "right_hand"; break; //[8]

	case	12:return	"left_hip"; break;
	case	13:return	"left_knee"; break;
	case	14:return	"left_ankle"; break;
	case	16:return	"right_hip"; break;
	case	17:return	"right_knee"; break;
	case	18:return	"right_ankle"; break;
	case	20:return	"shoulder_spine"; break;
	default:return "NULL";
	}

}

kinect_v2::BodyJoints& get_body(int n, kinect_v2::BodyJoints& person_0, kinect_v2::BodyJoints& person_1, kinect_v2::BodyJoints& person_2,
	kinect_v2::BodyJoints& person_3, kinect_v2::BodyJoints& person_4, kinect_v2::BodyJoints& person_5)
{
	//Return the specific kinect-v2::BodyJoint Object according the input index
	switch (n)
	{
	case	0:	return	person_0; break;
	case	1:	return	person_1; break;
	case	2:	return	person_2; break;
	case	3:	return	person_3; break;
	case	4:	return	person_4; break;
	case	5:	return	person_5; break;
	}

}

void reset_body(kinect_v2::BodyJoints& body)
{
	body.tracked = "NO";
	geometry_msgs::Pose* p = body.joints;
	for (int i = 0; i < sizeof(body.joints) / sizeof(geometry_msgs::Pose); ++i)
	{
		// string name = get_name(i);
		// const char *joint_name = name.c_str();
		p->position.x = 0;
		p->position.y = 0;
		p->position.z = 0;
		p->orientation.w = 1;
		p->orientation.x = 0;
		p->orientation.y = 0;
		p->orientation.z = 0;
		++p;
	}
}

void reset_joint(geometry_msgs::Pose& joint)
{
	joint.position.x = 0;
	joint.position.y = 0;
	joint.position.z = 0;
	joint.orientation.w = 1;
	joint.orientation.x = 0;
	joint.orientation.y = 0;
	joint.orientation.z = 0;
}


