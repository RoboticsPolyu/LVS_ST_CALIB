/** 
*********************************************************************
	Author        : Mohith Sakthivel
	Date          : 04.06.2019
	Description   : Windows console application to implement 
					EGM control Robot and Track Setup
*********************************************************************
**/

#include <thread>
#include "egm/egm_control.h"
#include "egm/haneye_path_control.h"
#include <cmath>

const std::string ROB1_Name = "ROB_1";
const int ROB1_PORT = 6510;

bool IsReadable = true;
// Input file parameters
int msg_count = 2500*10;
int sampling_time = 4;

std::chrono::time_point<std::chrono::steady_clock> StartTime, EndTime, CycleStartTime, PrevCycleStartTime;

std::ofstream test;
std::ofstream egm_debug;

float time_data[10] = { 0,0,0,0,0,0,0,0,0,0 };

float last_x = 0, last_y = 0;
inline void circle_path_pos(int count, Robot* sysRobot1 = nullptr)
{
	double x_p, y_p = 0;
	float x_f = 0, y_f = 0, z_f = 0;
	float eular_x = 0, eular_y, eular_z;
	float quat[4];

	sysRobot1->GetFeedbackPose(x_f, y_f, z_f, eular_x, eular_y, eular_z, quat);

	float radius = 100;

	float vel_y = 0, vel_x = 0;
	
	if(count <= 500)
	{
		vel_x = 0;
		vel_y = 50;
		y_p = 0 + vel_y * 0.004 * count;
		x_p = 0;
		last_x = x_p;
		last_y = y_p;
	}
	else if(count <= 1500 && count > 500)
	{
		x_p = radius* std::sin(0.36 / 180.0* M_PI*(count-500));
		y_p = radius* std::cos(0.36 / 180.0* M_PI*(count-500));
		vel_x = (x_p - last_x)/0.004;
		vel_y = (y_p - last_y)/0.004;
		
		last_x = x_p;
		last_y = y_p;
	}
	else if(count > 1500 && count <= 2000)
	{
		vel_x = 0;
		vel_y = -50;

		x_p = last_x ;
		y_p = last_y- 50* 0.004;
		last_y = y_p;
		last_x = x_p;

	}
	else
	{
		vel_x = 0;
		vel_y = 0;

		x_p = last_x;
		y_p = last_y;
		last_y = y_p;
		last_x = x_p;
	}

	sysRobot1->RobotPos[0] = x_p + sysRobot1->initRobotPos[0];
	sysRobot1->RobotPos[1] = y_p + sysRobot1->initRobotPos[1];
	sysRobot1->RobotPos[2] = sysRobot1->initRobotPos[2];

	egm_debug << count << " " << x_p << " " << x_f << " " << y_p << " " << y_f << " " << eular_x << " " << eular_y << " " << eular_z << std::endl;

}

void SetRobPos(Robot* sysRobot1, float x, float y, float z, float ex, float ey, float ez, float qw, float qx, float qy, float qz)
{
	sysRobot1->RobotPos[0] = x;
	sysRobot1->RobotPos[1] = y;
	sysRobot1->RobotPos[2] = z;

	sysRobot1->RobotEuler[0] = ex;
	sysRobot1->RobotEuler[1] = ey;
	sysRobot1->RobotEuler[2] = ez;

	sysRobot1->RobotQu[0] = qw;
	sysRobot1->RobotQu[1] = qx;
	sysRobot1->RobotQu[2] = qy;
	sysRobot1->RobotQu[3] = qz;
}

inline void circle_path_speed(int count, Robot* sysRobot1 = nullptr)
{
	double x_p, y_p = 0;
	float x_f = 0, y_f = 0, z_f = 0;
	float eular_x = 0, eular_y, eular_z;
	float quat[4];
	
	sysRobot1->GetFeedbackPose(x_f, y_f, z_f, eular_x, eular_y, eular_z, quat);

	float radius = 100;
	float vel_y = 0, vel_x = 0;
	
	if(count <= 500)
	{
		vel_x = 0;
		vel_y = 50;
		sysRobot1->RobotPosSpeed[0] = vel_x;
		sysRobot1->RobotPosSpeed[1] = vel_y;
		sysRobot1->RobotPosSpeed[2] = 0;

		y_p = 0 + vel_y * 0.004 * count;
		x_p = 0;
		last_x = x_p;
		last_y = y_p;
	}
	else if(count <= 1500 && count > 500)
	{
		x_p = radius* std::sin(0.36 / 180.0* M_PI*(count-500));
		y_p = radius* std::cos(0.36 / 180.0* M_PI*(count-500));
		vel_x = (x_p - last_x)/0.004;
		vel_y = (y_p - last_y)/0.004;
		
		// vel_x = (x_p - x_f - 451)/0.004;
		// vel_y = (y_p - y_f)/0.004; // ！！！！！！！！！！！！！！！！！！！！

		// if(vel_y > 200)
		// {
		// 	vel_y = 200;
		// }
		// if(vel_x > 200)
		// {
		// 	vel_x = 200;
		// }
		// if(vel_y < -200)
		// {
		// 	vel_y = -200;
		// }
		// if(vel_x < -200)
		// {
		// 	vel_x = -200;
		// }

		sysRobot1->RobotPosSpeed[0] = vel_x;
	 	sysRobot1->RobotPosSpeed[1] = vel_y;
	 	sysRobot1->RobotPosSpeed[2] = 0;
		
		last_x = x_p;
		last_y = y_p;
	}
	else if(count > 1500 && count <= 2000)
	{
		vel_x = 0;
		vel_y = -50;
		sysRobot1->RobotPosSpeed[0] = vel_x;
		sysRobot1->RobotPosSpeed[1] = vel_y;
		sysRobot1->RobotPosSpeed[2] = 0;
		x_p = last_x ;
		y_p = last_y- 50* 0.004;
		last_y = y_p;
		last_x = x_p;

	}
	else
	{
		vel_x = 0;
		vel_y = 0;
		sysRobot1->RobotPosSpeed[0] = vel_x;
		sysRobot1->RobotPosSpeed[1] = vel_y;
		sysRobot1->RobotPosSpeed[2] = 0;
		x_p = last_x;
		y_p = last_y;
		last_y = y_p;
		last_x = x_p;
	}

	egm_debug << count << " " << x_p << " " << x_f << " " << y_p << " " << y_f << " " << eular_x << " " << eular_y << " " << eular_z << std::endl;

}

int main(int argc, char* argv[]) {
	GOOGLE_PROTOBUF_VERIFY_VERSION;

	test.open("test log.txt", std::ios::trunc);
	egm_debug.open("egm_vel.txt", std::ios::trunc);

	Robot* Rob1 = new  Robot(ROB1_Name, ROB1_PORT);
	
	std::thread Robot1InitThread(&Robot::initRobot,Rob1);
	Robot1InitThread.join();
	float x_f = 0, y_f = 0, z_f = 0;
	float eular_x = 0, eular_y, eular_z;
	float quat[4];

	float linear_speed = 50;
	float radius = 100;
	float duration = 0.004f;
	HandeyePathPlaner handeye_path_planner;
	
	test << "Msg No, Total, Rob1 Read, Rob1 Write" << std::endl;
	//test << "Msg No, Total, Rob1 Read, Rob1 Write, Track1 Read, Track1 Write, Rob2 Read, Rob2 Write, Track2 Read, Track2 Write" << std::endl;
	
	while ((std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - Rob1->StreamStartTime).count() / 1e6) <= sampling_time) {}
	
	PrevCycleStartTime = CycleStartTime = StartTime = std::chrono::steady_clock::now();

	for (int i = 1; i <= msg_count; i++) {

		// Read feedback through protobuf
		std::thread ReadRobot1([](Robot* Rob1, int i, float* TimeData, bool IsReadable) {Rob1->FeedbackCycleCartesian(i, TimeData, IsReadable); },Rob1,i,&time_data[1],IsReadable);
		// wait for readthreads to join
		ReadRobot1.join();

		CycleStartTime = std::chrono::steady_clock::now();
		time_data[0] = float(std::chrono::duration_cast<std::chrono::nanoseconds>(CycleStartTime - PrevCycleStartTime).count() / 1e6);
		PrevCycleStartTime = CycleStartTime;
		
		Rob1->GetFeedbackPose(x_f, y_f, z_f, eular_x, eular_y, eular_z, quat);
		// circle_path_pos(i, Rob1);

		if(i == 1)
		{
			handeye_path_planner.InitFirstPoint(x_f, y_f, z_f, eular_x, eular_y, eular_z, linear_speed, radius);
		}
		HandeyePathPlaner::path_point planed_path_point;
		handeye_path_planner.PlanTest(duration, planed_path_point);
		SetRobPos(Rob1, planed_path_point.x, planed_path_point.y, planed_path_point.z, planed_path_point.eular_x, planed_path_point.eular_y, planed_path_point.eular_z, 
			planed_path_point.qw, planed_path_point.qx, planed_path_point.qy, planed_path_point.qz);

		egm_debug << planed_path_point.x << " " << planed_path_point.y << " " << planed_path_point.z << " " << planed_path_point.eular_x << " " 
			<< planed_path_point.eular_y << " " << planed_path_point.eular_z << " " << planed_path_point.qw << " " << planed_path_point.qx << " "
			<< planed_path_point.qy << " " << planed_path_point.qz << " " 
			<< x_f << " " << y_f << " " << z_f << " " << eular_x << " " << eular_y << " " << eular_z << " "
			<< quat[0] << " " << quat[1] << " " << quat[2] << " " << quat[3] << std::endl;

		// Send data through Protobuf
		std::thread WriteRobot1([](Robot* Rob1, float* TimeData) {Rob1->WriteCycleCartesian(TimeData); }, Rob1, &time_data[2]);
		// wait for read and write threads to join
		WriteRobot1.join();

		// Hold the loop for sampling time
		// while ((std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - CycleStartTime).count() / 1e6) <= sampling_time) {}
	}

	EndTime = std::chrono::steady_clock::now();
	std::cout << std::endl <<"Expected Runtime:" << ((msg_count*sampling_time)/1000.0) << std::endl <<
		"Execution Time:" << std::chrono::duration_cast<std::chrono::microseconds>(EndTime - StartTime).count() / 1e6 << " seconds" << std::endl;
	test << std::endl << std::endl << "Expected Runtime:" << ((msg_count * sampling_time) / 1000.0) << std::endl <<
		"Execution Time:" << std::chrono::duration_cast<std::chrono::microseconds>(EndTime - StartTime).count() / 1e6 << " seconds" << std::endl;

	delete Rob1;
	std::cin.get();
	test.close();
	return 0;
}