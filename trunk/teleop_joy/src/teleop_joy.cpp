#include <ros/ros.h>
#include <joy/Joy.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <boost/thread.hpp> 
#include <boost/date_time.hpp> 
#include <boost/date_time/posix_time/posix_time.hpp>

class TeleopTurtle
{
public:
  TeleopTurtle();
  void repeat();
  
private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  
  ros::NodeHandle nh_;

  int linear_, angular_;
  double l_scale_, a_scale_;
  ros::Subscriber joy_sub_;
  
};

ros::Publisher vel_pub_;

TeleopTurtle::TeleopTurtle():
  linear_(1),
  angular_(2)
{

  nh_.param("axis_linear", linear_, linear_);
  nh_.param("axis_angular", angular_, angular_);
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);


  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopTurtle::joyCallback, this);

}

double go_ahead = 0;
double turn = 0;

void repeat()
{
	while(true) {
		if (
			go_ahead > 0.05 || go_ahead < -0.05 ||
			turn > 0.05 || turn < -0.05
			)
		{
			
			//std::cout << "Repeat linear velocity: " << go_ahead << std::endl;  
			geometry_msgs::Twist vel;
			if (go_ahead > 0.05 || go_ahead < -0.05)
				vel.linear.x = go_ahead;
			if (turn > 0.05 || turn < -0.05)
				vel.angular.z = turn;
			vel_pub_.publish(vel);
			
		} // else std::cout << "Skip " << std::endl;
		
		boost::this_thread::sleep(boost::posix_time::milliseconds(250)); 
	}
}

void TeleopTurtle::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  geometry_msgs::Twist vel;
  vel.angular.z = a_scale_* joy->axes[angular_];
  vel.linear.x = l_scale_* joy->axes[linear_];
  go_ahead = l_scale_* joy->axes[linear_];
  turn = a_scale_* joy->axes[angular_];
  //std::cout << "go_ahead is " << go_ahead << std::endl;
  vel_pub_.publish(vel);
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "teleop_turtle");
	TeleopTurtle teleop_turtle;

	boost::thread workerThread(repeat);  
	ros::spin();
}
