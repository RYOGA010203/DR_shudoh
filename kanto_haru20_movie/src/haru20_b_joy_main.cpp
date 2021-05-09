/*
 * kari.cpp
 *
 *  Created on: Aug 19, 2019
 *      Author: shun
 */

#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Joy.h>
#include <vector>
#include <string>

#define pi 3.1416

enum class CarrierStatus
    : uint16_t
    {
        shutdown = 0x0000,
    reset = 0x0001,

/*
 operational			= 0x0002,

 chuck0_chucked		= 0x0010,
 chuck1_chucked		= 0x0020,
 chuck2_chucked		= 0x0040,
 chuck3_chucked		= 0x0080,
 */
};

enum class CarrierCommands
    : uint16_t
    {
        shutdown_cmd = 0x0000,
    reset_cmd = 0x0001,
    /*
     operational			= 0x0002,
     */

    chuck_cmd = 0x0100,
    unchuck_cmd = 0x0200,

    chuck0 = 0x0010,
    chuck1 = 0x0020,
    chuck2 = 0x0040,
    chuck3 = 0x0080,
};

class CrMain
{
public:
    CrMain(void);

private:
    void shutdownInputCallback(const std_msgs::Empty::ConstPtr& msg);
    void startInputCallback(const std_msgs::Empty::ConstPtr& msg);
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
	void PosRealCallback(const std_msgs::Float32::ConstPtr& msg);

    ros::NodeHandle nh_;

    //int linear_, angular_;
    ros::Subscriber joy_sub;
    ros::Subscriber shutdown_input_sub;
    ros::Subscriber start_input_sub;

	/***********************/
	ros::Subscriber throw_pos_real_sub;
	/***********************/

    ros::Publisher pick_position_pub;
    ros::Publisher pick_enable_pub;

    ros::Publisher throw_target_pub;
    ros::Publisher throw_enable_pub;

	/***********************/
	ros::Publisher solenoid_position_pub;
    ros::Publisher solenoid_enable_pub;
	/***********************/

    ros::Publisher act_enable_pub0;
    ros::Publisher act_enable_pub1;
    ros::Publisher act_enable_pub2;
	/**********************/
	ros::Publisher act_enable_pub3;
	/**********************/

    std_msgs::Float32 pick_position_msg;
    std_msgs::Float32 throw_position_msg;
	std_msgs::Float32 throw_velocity_msg;
	/**********************/
	std_msgs::Float32 throw_pos_real_msg;
	/**********************/

	/**********************/
	std_msgs::UInt8 solenoid_position_msg;
	/**********************/
    std_msgs::UInt8 act_enable_msg;

	/**********************/
	std_msgs::UInt8 shirasu_cmd_msg;
	/**********************/

    double steps_per_mm = 1;

    std::vector<double> pick_position = { 0, 2*pi, -2*pi, 6*pi};
    int pick_position_index = 0;

	std::vector<double> throw_velocity = { 0, 2*pi, -2*pi, 4*pi, 8*pi};  //vel
    int throw_velocity_index = 0;

    std::vector<double> throw_position = { 0, 0.5*pi, pi, 1.5*pi, 2*pi}; //pos
    int throw_position_index = 0;

	double throw_position_real = 0;

	int dr_mode = 0;
	// 0 : taiki_mode
	// 1 : genten_awase_mode
	// 2 : haji_mode
	// 3 : tohteki_mode
	// 4 : yoin_mode
	// 5 : defence_mode

	/************************/
	unsigned char solenoid_data = 0;
	/************************/

    //		{0, -40 * steps_per_mm;
    //static constexpr int lift_position_first = -40 * steps_per_mm;
    //static constexpr int lift_position_second = lift_position_first - (248 * steps_per_mm);
    //static constexpr int lift_position_third = lift_position_second - (248 * steps_per_mm);

    bool _shutdown = true;

    static int Button1;
    static int Button2;
    static int Button3;
    static int Button4;
    static int Button5;
    static int Button6;
    static int Button11;
    static int Button12;
	/***************/
	static int Button9;
    static int Button10;
	/***************/
    static int Button7;
    static int Button8;

    static int AxisDPadX;
    static int AxisDPadY;
};

int CrMain::Button1 = 0;
int CrMain::Button2 = 1;
int CrMain::Button3 = 2;
int CrMain::Button4 = 3;
int CrMain::Button5 = 4;
int CrMain::Button6 = 5;
int CrMain::Button7 = 6;
int CrMain::Button8 = 7;
/******************/
int CrMain::Button9 = 8;
int CrMain::Button10 = 9;
/******************/
int CrMain::Button11 = 10;
int CrMain::Button12 = 11;

int CrMain::AxisDPadX = 4;
int CrMain::AxisDPadY = 5;

CrMain::CrMain(void)
{
    joy_sub = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &CrMain::joyCallback, this);
    shutdown_input_sub = nh_.subscribe<std_msgs::Empty>("shutdown_input", 10, &CrMain::shutdownInputCallback, this);
    start_input_sub = nh_.subscribe<std_msgs::Empty>("start_input", 10, &CrMain::startInputCallback, this);

	/***************************************/
	throw_pos_real_sub = nh_.subscribe<std_msgs::Float32>("throw/motorth_pos_real", 10, &CrMain::PosRealCallback, this);
	/***************************************/

    this->throw_target_pub = nh_.advertise<std_msgs::Float32>("throw/motorth_target", 1);
    this->throw_enable_pub = nh_.advertise<std_msgs::UInt8>("throw/motorth_cmd", 1);

    this->pick_position_pub = nh_.advertise<std_msgs::Float32>("pick/motorpc_cmd_pos", 1);
    this->pick_enable_pub = nh_.advertise<std_msgs::UInt8>("pick/motorpc_cmd", 1);

	/**************************************************/
	this->solenoid_position_pub = nh_.advertise<std_msgs::UInt8>("solenoid/motorsl_cmd_pos", 1);
    this->solenoid_enable_pub = nh_.advertise<std_msgs::UInt8>("solenoid/motorsl_cmd", 1);
	/**************************************************/

    this->act_enable_pub0 = nh_.advertise<std_msgs::UInt8>("base/motor0_cmd", 1);
    this->act_enable_pub1 = nh_.advertise<std_msgs::UInt8>("base/motor1_cmd", 1);
    this->act_enable_pub2 = nh_.advertise<std_msgs::UInt8>("base/motor2_cmd", 1);
	/**************************************************/
	this->act_enable_pub3 = nh_.advertise<std_msgs::UInt8>("base/motor3_cmd", 1);
	/**************************************************/
    //this->hand_unchuck_thres_pub = nh_.advertise<std_msgs::UInt16>("hand/unchuck_thres", 1);

    auto nh_priv = ros::NodeHandle("~");

    /*nh_priv.getParam("lift_step_per_mm", this->steps_per_mm);

    std::vector<double> tmp;
    nh_priv.getParam("pick_position", tmp);
    if (tmp.size() == 5)
    {
        this->pick_position = tmp;
    }

    for (double& pos : this->pick_position)
    {
        pos *= (-steps_per_mm);
    }

    ROS_INFO("pick_pos: %f, %f, %f, %f, %f", this->pick_position[0], this->pick_position[1], this->pick_position[2],
            this->pick_position[3], this->pick_position[4]);*/

	/************************************************************
	std::vector<int> tmp2;
    nh_priv.getParam("solenoid_position", tmp2);
    if (tmp2.size() == 5)
    {
        this->solenoid_position = tmp2;
    }

    for (int& pos : this->solenoid_position)
    {
        pos *= (-steps_per_mm);
    }

    ROS_INFO("solenoid_pos: %d, %d, %d, %d, %d", this->solenoid_position[0], this->solenoid_position[1], this->solenoid_position[2],
            this->solenoid_position[3], this->solenoid_position[4]);
	************************************************************/

    nh_.getParam("Button1", Button1);
    nh_.getParam("Button2", Button2);
    nh_.getParam("Button3", Button3);
    nh_.getParam("Button4", Button4);
    nh_.getParam("Button5", Button5);
    nh_.getParam("Button6", Button6);
    nh_.getParam("Button11", Button11);
    nh_.getParam("Button12", Button12);
	/***************************/
	nh_.getParam("Button9", Button9);
    nh_.getParam("Button10", Button10);
	/***************************/

    nh_.getParam("Button7", Button7);
    nh_.getParam("Button8", Button8);

    nh_.getParam("AxisDPadX", AxisDPadX);
    nh_.getParam("AxisDPadY", AxisDPadY);

}

void CrMain::shutdownInputCallback(const std_msgs::Empty::ConstPtr& msg)
{
    if (!this->_shutdown)
    {
//        this->_shutdown = 1;

//        ROS_INFO("aborting.");
    }

    // reset this:
    // this->CurrentCommandIndex = -1;
    pick_position_index = 0;
}

void CrMain::startInputCallback(const std_msgs::Empty::ConstPtr& msg)
{
    // bring the robot back operational

    ROS_INFO("starting.");

    act_enable_msg.data = 1;
    act_enable_pub0.publish(act_enable_msg);
    act_enable_pub1.publish(act_enable_msg);
    act_enable_pub2.publish(act_enable_msg);
	act_enable_pub3.publish(act_enable_msg);
    pick_enable_pub.publish(act_enable_msg);
    throw_enable_pub.publish(act_enable_msg);
	/******************************/
	solenoid_enable_pub.publish(act_enable_msg);
	/******************************/
    this->_shutdown = 0;
}
/**************************************************************************************/
void CrMain::PosRealCallback(const std_msgs::Float32::ConstPtr& msg)
{
	throw_position_real = msg->data;
	if(dr_mode == 3)
	{
		if(throw_position_real > throw_position[4])
		{
			dr_mode = 4;
			solenoid_data = (solenoid_data & 0b11111011);
            solenoid_position_msg.data = solenoid_data;
            solenoid_position_pub.publish(solenoid_position_msg);

            ROS_INFO("solenoid ch2 off");
			ROS_INFO("dr_mode = 4");
		}
	}
	if(dr_mode == 4)
	{
		if(throw_position_real > ( throw_position[4] + pi/2 ) )
		{
			dr_mode = 0;
			throw_velocity_index = 0;
            throw_velocity_msg.data = throw_velocity[throw_velocity_index];
            throw_target_pub.publish(throw_velocity_msg);

			ROS_INFO("vel Index 0");
			ROS_INFO("dr_mode = 0");
			ROS_INFO("finished");
		}
	}

}
/**************************************************************************************/
void CrMain::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    static bool last_1 = false;
    static bool last_2 = false;
    static bool last_3 = false;
    static bool last_4 = false;
    static bool last_5 = false;
    static bool last_6 = false;
    static bool last_11 = false;
    static bool last_12 = false;
    static bool last_7 = false;
    static bool last_8 = false;
	/****************/
	static bool last_9 = false;
    static bool last_10 = false;
	/****************/

	static char last_13 = false;
	static char last_14 = false;


    //static int last_dpadXCmd = 0;

    bool _1 = joy->buttons[Button1];
    bool _2 = joy->buttons[Button2];
    bool _3 = joy->buttons[Button3];
    bool _4 = joy->buttons[Button4];
    bool _5 = joy->buttons[Button5];
    bool _6 = joy->buttons[Button6];
    bool _11 = joy->buttons[Button11];
    bool _12 = joy->buttons[Button12];
	/****************/
	bool _9 = joy->buttons[Button9];
    bool _10 = joy->buttons[Button10];
	/****************/
    bool _7 = joy->buttons[Button7];
    bool _8 = joy->buttons[Button8];

	char _14 = joy->axes[AxisDPadX];
    char _13 = joy->axes[AxisDPadY];

    if (_12 && !last_12)
    {
        ROS_INFO("all enable          (button12:AxisRightTrigger)");

        act_enable_msg.data = 1;
        act_enable_pub0.publish(act_enable_msg);
        act_enable_pub1.publish(act_enable_msg);
        act_enable_pub2.publish(act_enable_msg);
		act_enable_pub3.publish(act_enable_msg);
        pick_enable_pub.publish(act_enable_msg);
        throw_enable_pub.publish(act_enable_msg);
		/****************************/
		solenoid_enable_pub.publish(act_enable_msg);
		/****************************/

        this->_shutdown = 0;  
    }


    if (_11 && !last_11)
    {
        if (!this->_shutdown)
        {
            this->_shutdown = 1;

            ROS_INFO("all disable & stop  (button11:AxisLeftTrigger)");
        }
        
        act_enable_msg.data = 0;
        act_enable_pub0.publish(act_enable_msg);
        act_enable_pub1.publish(act_enable_msg);
        act_enable_pub2.publish(act_enable_msg);
		act_enable_pub3.publish(act_enable_msg);
        pick_enable_pub.publish(act_enable_msg);
        throw_enable_pub.publish(act_enable_msg);
		/************************/
		solenoid_enable_pub.publish(act_enable_msg);
		/************************/
    }

    if (!this->_shutdown)
    {
        if (_7 && !last_7)
        {
            /*// receive  the throw
            throw_position_index = 1;
            throw_position_msg.data = throw_position[throw_position_index];
            throw_position_pub.publish(throw_position_msg);*/

			ROS_INFO("rotation Left       (button7:ButtonLeftThumb)");
        }
        else if (_8 && !last_8)
        {
            /*//  the throw
            throw_position_index = 2;
            throw_position_msg.data = throw_position[throw_position_index];
            throw_position_pub.publish(throw_position_msg);*/

			ROS_INFO("rotation Right      (button8:ButtonRightThumb)");
        }
		else if (_4 && !last_4)
        {
			/*if ( (solenoid_data & 0b00001000) == 0b00000000 )
        	{
            	// solenoid ch3 on
            	solenoid_data = (solenoid_data | 0b00001000);
            	solenoid_position_msg.data = solenoid_data;
            	solenoid_position_pub.publish(solenoid_position_msg);

            	ROS_INFO("solenoid ch3 on     (button4:ButtonY)");
            }
            
            else
            {
            	// solenoid ch3 off
            	solenoid_data = (solenoid_data & 0b11110111);
            	solenoid_position_msg.data = solenoid_data;
            	solenoid_position_pub.publish(solenoid_position_msg);

            	ROS_INFO("solenoid ch3 off    (button4:ButtonY)");
            }*/

			// solenoid_ch3 is now used for velocity special data access button

			throw_position[0] = throw_position_real;
			throw_position[1] = throw_position_real + 0.5*pi ;
			throw_position[2] = throw_position_real + pi ;
			throw_position[3] = throw_position_real + 1.5*pi ;
			throw_position[4] = throw_position_real + 2*pi ;
			ROS_INFO("position vector update");

        }
        else if (_3 && !last_3)
        {
			if ( (solenoid_data & 0b00000001) == 0b00000000 )
        	{
            	// solenoid ch0 on
            	solenoid_data = (solenoid_data | 0b00000001);
            	solenoid_position_msg.data = solenoid_data;
            	solenoid_position_pub.publish(solenoid_position_msg);

            	ROS_INFO("solenoid ch0 on     (button3:ButtonB)");
            }
            
            else
            {
            	// solenoid ch0 off
            	solenoid_data = (solenoid_data & 0b11111110);
            	solenoid_position_msg.data = solenoid_data;
            	solenoid_position_pub.publish(solenoid_position_msg);

            	ROS_INFO("solenoid ch0 off    (button3:ButtonB)");
            }
        }
        else if (_2 && !last_2)
        {
			if ( (solenoid_data & 0b00000010) == 0b00000000 )
        	{
            	// solenoid ch1 on
            	solenoid_data = (solenoid_data | 0b00000010);
            	solenoid_position_msg.data = solenoid_data;
            	solenoid_position_pub.publish(solenoid_position_msg);

            	ROS_INFO("solenoid ch1 on     (button2:ButtonA)");
            }
            
            else
            {
            	// solenoid ch1 off
            	solenoid_data = (solenoid_data & 0b11111101);
            	solenoid_position_msg.data = solenoid_data;
            	solenoid_position_pub.publish(solenoid_position_msg);

            	ROS_INFO("solenoid ch1 off    (button2:ButtonA)");
            }
        }
        else if (_1 && !last_1)
        {
			if ( (solenoid_data & 0b00000100) == 0b00000000 )
        	{
            	// solenoid ch2 on
            	solenoid_data = (solenoid_data | 0b00000100);
            	solenoid_position_msg.data = solenoid_data;
            	solenoid_position_pub.publish(solenoid_position_msg);

            	ROS_INFO("solenoid ch2 on     (button1:ButtonX)");
            }
            
            else
            {
            	// solenoid ch2 off
            	solenoid_data = (solenoid_data & 0b11111011);
            	solenoid_position_msg.data = solenoid_data;
            	solenoid_position_pub.publish(solenoid_position_msg);

            	ROS_INFO("solenoid ch2 off    (button1:ButtonX)");
            }
        }
		/******************************************/
		else if (_5 && !last_5)
        {
			shirasu_cmd_msg.data = 1;
			throw_enable_pub.publish(shirasu_cmd_msg);

			ROS_INFO("shirasu vel         (button5:ButtonLB)");
        }
        else if (_6 && !last_6)
        {
            shirasu_cmd_msg.data = 6;
			throw_enable_pub.publish(shirasu_cmd_msg);

			ROS_INFO("shirasu pos         (button6:ButtonRB)");
			ROS_INFO("WARNING !!  IN this state, you shuold be aware of the miss click!!");
        }
		/*******************************************/
        else if (_13 && !last_13)
        {
            if(_13 == 1)
			{
				if(shirasu_cmd_msg.data == 1)
				{
					// velocity Index 3
            		throw_velocity_index = 3;
            		throw_velocity_msg.data = throw_velocity[throw_velocity_index];
            		throw_target_pub.publish(throw_velocity_msg);

            		ROS_INFO("vel Index 3         (button13+:AxisDPadX)");
				}
				else if(shirasu_cmd_msg.data == 6)
				{
					// position Index 3
            		throw_position_index = 3;
            		throw_position_msg.data = throw_position[throw_position_index];
            		throw_target_pub.publish(throw_position_msg);

            		ROS_INFO("pos Index 3         (button13+:AxisDPadX)");
				}
			}
			else if(_13 == -1)
			{
				if(shirasu_cmd_msg.data == 1)
				{
					// velocity Index 1
            		throw_velocity_index = 1;
            		throw_velocity_msg.data = throw_velocity[throw_velocity_index];
            		throw_target_pub.publish(throw_velocity_msg);

            		ROS_INFO("vel Index 1         (button13-:AxisDPadX)");
				}
				else if(shirasu_cmd_msg.data == 6)
				{
					// position Index 1
            		throw_position_index = 1;
            		throw_position_msg.data = throw_position[throw_position_index];
            		throw_target_pub.publish(throw_position_msg);

            		ROS_INFO("pos Index 1         (button13-:AxisDPadX)");
				}
			}
        }
        else if (_14 && !last_14)
        {
            if(_14 == 1)
			{
				if(shirasu_cmd_msg.data == 1)
				{
					// velocity Index 2
            		throw_velocity_index = 2;
            		throw_velocity_msg.data = throw_velocity[throw_velocity_index];
            		throw_target_pub.publish(throw_velocity_msg);

            		ROS_INFO("vel Index 2         (button14+:AxisDPadY)");
				}
				else if(shirasu_cmd_msg.data == 6)
				{
					// position Index 2
            		throw_position_index = 2;
            		throw_position_msg.data = throw_position[throw_position_index];
            		throw_target_pub.publish(throw_position_msg);

            		ROS_INFO("pos Index 2         (button14+:AxisDPadY)");
				}
			}
			else if(_14 == -1)
			{
				if(shirasu_cmd_msg.data == 1)
				{
					// velocity Index 0
            		throw_velocity_index = 0;
            		throw_velocity_msg.data = throw_velocity[throw_velocity_index];
            		throw_target_pub.publish(throw_velocity_msg);

            		ROS_INFO("vel Index 0         (button14-:AxisDPadY)");
				}
				else if(shirasu_cmd_msg.data == 6)
				{
					// position Index 0
            		throw_position_index = 0;
            		throw_position_msg.data = throw_position[throw_position_index];
            		throw_target_pub.publish(throw_position_msg);

            		ROS_INFO("pos Index 0         (button14-:AxisDPadY)");
				}
			}
        }
		/*********************************************/

		else if (_9 && !last_9)
        {
			if(shirasu_cmd_msg.data == 1)
			{
				if(act_enable_msg.data == 1)
				{
					act_enable_msg.data = 2;
					solenoid_enable_pub.publish(act_enable_msg);
					act_enable_msg.data = 1;
					ROS_INFO("ashidome on         (button9:ButtonBack)");
					ROS_INFO("This mode will be automatically reseted");
					ROS_INFO("Please input little velocity data");
				}
			}
        }
		else if (_10 && !last_10)
        {
			if(shirasu_cmd_msg.data == 1)
			{
				throw_velocity_index = 4;
            	throw_velocity_msg.data = throw_velocity[throw_velocity_index];
           		throw_target_pub.publish(throw_velocity_msg);

				dr_mode = 3;

				ROS_INFO("HI SEEPD !!  BECAREFUL !!");
				ROS_INFO("dr_mode 3");
           		ROS_INFO("vel Index 4         (button10:ButtonStart)");

				//dr_mode = 3;	// tohteki_mode
			}
        }
		/********************************************/
    }


    last_1 = _1;
    last_2 = _2;
    last_3 = _3;
    last_4 = _4;
    last_5 = _5;
    last_6 = _6;
    last_7 = _7;
    last_8 = _8;
	/******/
	last_9 = _9;
    last_10 = _10;
	/******/
    last_11 = _11;
    last_12 = _12;

	last_13 = _13;
	last_14 = _14;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "natu19_b_joy_main");

    CrMain *crMain = new CrMain();
    ROS_INFO("natu19_b_joy_main node has started.");

    ros::spin();
    ROS_INFO("natu19_b_joy_main node has been terminated.");
}

