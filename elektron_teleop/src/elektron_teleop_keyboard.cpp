#include <termios.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#define KEYCODE_A 0x61
#define KEYCODE_D 0x64
#define KEYCODE_S 0x73
#define KEYCODE_W 0x77

#define KEYCODE_A_CAP 0x41
#define KEYCODE_D_CAP 0x44
#define KEYCODE_S_CAP 0x53
#define KEYCODE_W_CAP 0x57

#define KEYCODE_SPC 0x20

class ElektronTeleopKeyboard {
private:
	geometry_msgs::Twist vel_;

	ros::NodeHandle nh_;

	double l_scale_, a_scale_, walk_scale_;
	ros::Publisher vel_pub_;

public:
	void init() {

		nh_.param("scale_angular", a_scale_, 1.0);
		nh_.param("scale_linear", l_scale_, 0.23);
		nh_.param("walk_scale", walk_scale_, 0.6);

		vel_pub_ = nh_.advertise<geometry_msgs::Twist> ("cmd_vel", 1);

		ros::NodeHandle n_private("~");
	}

	void keyboardLoop();
};

int kfd = 0;
struct termios cooked, raw;

void quit(int sig) {
	tcsetattr(kfd, TCSANOW, &cooked);
	exit(0);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "elektron_teleop_keyboard");

	ElektronTeleopKeyboard tpk;
	tpk.init();

	signal(SIGINT, quit);

	tpk.keyboardLoop();

	return (0);
}

void ElektronTeleopKeyboard::keyboardLoop() {
	char c;
	bool dirty = false;

	// get the console in raw mode
	tcgetattr(kfd, &cooked);
	memcpy(&raw, &cooked, sizeof(struct termios));
	raw.c_lflag &= ~(ICANON | ECHO);
	// Setting a new line, then end of file
	raw.c_cc[VEOL] = 1;
	raw.c_cc[VEOF] = 2;
	tcsetattr(kfd, TCSANOW, &raw);

	puts("Reading from keyboard");
	puts("---------------------------");
	puts("Use 'WS' to forward/back");
	puts("Use 'AD' to turn left/right");
	puts("Press 'Shift' to run");
	puts("Press 'Space' to stop");


	for (;;) {
		// get the next event from the keyboard
		if (read(kfd, &c, 1) < 0) {
			perror("read():");
			exit(-1);
		}
		
		vel_.linear.x = 0;
		vel_.angular.z = 0;

		switch (c) {
		// Walking
		case KEYCODE_W:
			vel_.linear.x = l_scale_ * walk_scale_;
			dirty = true;
			break;
		case KEYCODE_S:
			vel_.linear.x = -l_scale_ * walk_scale_;
			dirty = true;
			break;
		case KEYCODE_A:
			vel_.angular.z = a_scale_ * walk_scale_;
			dirty = true;
			break;
		case KEYCODE_D:
			vel_.angular.z = -a_scale_ * walk_scale_;
			dirty = true;
			break;

		// Running
		case KEYCODE_W_CAP:
			vel_.linear.x = l_scale_;
			dirty = true;
			break;
		case KEYCODE_S_CAP:
			vel_.linear.x = -l_scale_;
			dirty = true;
			break;
		case KEYCODE_A_CAP:
			vel_.angular.z = a_scale_;
			dirty = true;
			break;
		case KEYCODE_D_CAP:
			vel_.angular.z = -a_scale_;
			dirty = true;
			break;

		// Stop on space
		case KEYCODE_SPC:
			vel_.linear.x = 0;
			vel_.angular.z = 0;
			dirty = true;
			break;
		}

		if (dirty == true) {
			vel_pub_.publish(vel_);
		}

	}
}
