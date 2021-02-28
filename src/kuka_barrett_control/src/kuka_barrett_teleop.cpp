/**
 * Created by karadalex on 28/2/21.
 * 
 * Code based on https://github.com/ros/ros_tutorials/blob/noetic-devel/turtlesim/tutorials/teleop_turtle_key.cpp
 * No windows support
 */

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <signal.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>

// https://www.asciitable.com/
#define KEYCODE_RIGHT 0x43
#define KEYCODE_LEFT 0x44
#define KEYCODE_UP 0x41
#define KEYCODE_DOWN 0x42
#define KEYCODE_A 0x61
#define KEYCODE_B 0x62
#define KEYCODE_C 0x63
#define KEYCODE_D 0x64
#define KEYCODE_E 0x65
#define KEYCODE_F 0x66
#define KEYCODE_G 0x67
#define KEYCODE_Q 0x71
#define KEYCODE_R 0x72
#define KEYCODE_S 0x73
#define KEYCODE_T 0x74
#define KEYCODE_V 0x76

class KeyboardReader
{
public:
	KeyboardReader()
	{
		// get the console in raw mode
		tcgetattr(kfd, &cooked);
		struct termios raw;
		memcpy(&raw, &cooked, sizeof(struct termios));
		raw.c_lflag &= ~(ICANON | ECHO);
		// Setting a new line, then end of file
		raw.c_cc[VEOL] = 1;
		raw.c_cc[VEOF] = 2;
		tcsetattr(kfd, TCSANOW, &raw);
	}
	void readOne(char *c)
	{
		int rc = read(kfd, c, 1);
		if (rc < 0)
		{
			throw std::runtime_error("read failed");
		}
	}
	void shutdown()
	{
		tcsetattr(kfd, TCSANOW, &cooked);
	}

private:
	int kfd;
	struct termios cooked;
};

KeyboardReader input;

class KeyboardTeleopControl
{
public:
	KeyboardTeleopControl();
	void keyLoop();

private:
	ros::NodeHandle nh_;
	double x, y, z, roll, pitch, yaw;
	ros::Publisher twist_pub_;
};

KeyboardTeleopControl::KeyboardTeleopControl()
{
	twist_pub_ = nh_.advertise<geometry_msgs::Twist>("kuka_barrett/cmd_vel", 1);
}

void quit(int sig)
{
	(void)sig;
	input.shutdown();
	ros::shutdown();
	exit(0);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "kuka_barrett_teleop");
	KeyboardTeleopControl kuka_barrett_teleop;

	signal(SIGINT, quit);

	kuka_barrett_teleop.keyLoop();
	quit(0);

	return (0);
}

void KeyboardTeleopControl::keyLoop()
{
	char c;
	bool dirty = false;

	puts("Reading from keyboard");
	puts("---------------------------");
	puts("[left/right]  Use arrow keys to move the robot in x axis");
	puts("[up/down]     Use arrow keys to move the robot in z axis");
	puts("[a/s]         Use arrow keys to move the robot in y axis");
	puts("[e/r]         Use arrow keys to move the robot in roll axis");
	puts("[d/f]         Use arrow keys to move the robot in pitch axis");
	puts("[c/v]         Use arrow keys to move the robot in yaw axis");
	puts("[q]           Quit.");

	for (;;)
	{
		// get the next event from the keyboard
		try
		{
			input.readOne(&c);
		}
		catch (const std::runtime_error &)
		{
			perror("read():");
			return;
		}

		x = y = z = roll = pitch = yaw = 0;
		ROS_DEBUG("value: 0x%02X\n", c);

		switch (c)
		{
		case KEYCODE_LEFT:
			x = -1.0;
			dirty = true;
			break;
		case KEYCODE_RIGHT:
			x = 1.0;
			dirty = true;
			break;
		case KEYCODE_UP:
			z = 1.0;
			dirty = true;
			break;
		case KEYCODE_DOWN:
			z = -1.0;
			dirty = true;
			break;
		case KEYCODE_A:
			y = -1.0;
			dirty = true;
			break;
		case KEYCODE_S:
			y = 1.0;
			dirty = true;
			break;
		case KEYCODE_E:
			roll = -1.0;
			dirty = true;
			break;
		case KEYCODE_R:
			roll = 1.0;
			dirty = true;
			break;
		case KEYCODE_D:
			pitch = -1.0;
			dirty = true;
			break;
		case KEYCODE_F:
			pitch = 1.0;
			dirty = true;
			break;
		case KEYCODE_C:
			yaw = -1.0;
			dirty = true;
			break;
		case KEYCODE_V:
			yaw = 1.0;
			dirty = true;
			break;
		case KEYCODE_Q:
			ROS_DEBUG("quit");
			return;
		}

		geometry_msgs::Twist twist;
		twist.linear.x = x;
		twist.linear.y = y;
		twist.linear.z = z;
		twist.angular.x = roll;
		twist.angular.y = pitch;
		twist.angular.z = yaw;

		if (dirty)
		{
			twist_pub_.publish(twist);
			dirty = false;
		}
	}

	return;
}
