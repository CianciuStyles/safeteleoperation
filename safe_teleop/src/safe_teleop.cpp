#include <sstream>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "gradient_map/GradientMap.h"
#include <math.h>

/* libjoyrumble */

std::string joy_dev_;

/*  Includes */
#include <stdlib.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <dirent.h>
#include <fcntl.h>
#include <pthread.h> 
#include <sys/ioctl.h>
#include <linux/input.h>
#include <errno.h>

/* test_bit  : Courtesy of Johan Deneux */
#define BITS_PER_LONG (sizeof(long) * 8)
#define OFF(x)  ((x)%BITS_PER_LONG)
#define BIT(x)  (1UL<<OFF(x))
#define LONG(x) ((x)/BITS_PER_LONG)
#define test_bit(bit, array)    ((array[LONG(bit)] >> OFF(bit)) & 1)


/* 8 joypads should be enough for everyone */
#define MAXJOY 8


/* Shared variables */
static unsigned long features[4];
static struct ff_effect effects[MAXJOY];
static struct input_event stop[MAXJOY];
static struct input_event play[MAXJOY];
static char *eventfile[MAXJOY];
static int hasrumble[MAXJOY];
static int int_joynumber;
static int int_strong[MAXJOY];
static int int_weak[MAXJOY];
static int int_duration[MAXJOY];
static int mtlock=0;
static int initialized=0;
static int lastjoy=-1;


/* Check if a file exists */
int file_exists(char *filename)
{
	FILE *i;
  i = fopen(filename, "r");

if (i == NULL)
  {
    return 0;
  }
  
  fclose(i);
  return 1;
  
}


/* Check if a directory exists */
int dir_exists(char *filename)
{
	DIR *i ;
	i =opendir(filename);
	
	if (i == NULL)
	{
	return 0;
	}
	
	closedir(i);
	return 1;
}



/* Initialization. This should run automatically. */
void __attribute__ ((constructor)) joyrumble_init(void)
{
	
	#ifdef DEBUG
		printf("INIT\n");
	#endif
	
	/* event_fd is the handle of the event file */
	int event_fd;
	
	int count=0, j=0;
	
	/* this will hold the path of files while we do some checks */
	char tmp[128];
	
	for (count = 0; count < MAXJOY; count++)
	{
		
		sprintf(tmp,"/dev/input/js%d",count);
		/* Check if joystick device exists */
		if (file_exists(tmp)==1){
			lastjoy=count;	
			for (j = 0; j <= 99; j++)
			{
			/* Try to discover the corresponding event number */
			sprintf(tmp,"/sys/class/input/js%d/device/event%d",count,j);
			if (dir_exists(tmp)==1){
				/* Store the full path of the event file in the eventfile[] array */
				sprintf(tmp,"/dev/input/event%d",j);
				eventfile[count]=(char *)calloc(sizeof(tmp),sizeof(char));
				sprintf(eventfile[count], "%s", tmp);
				#ifdef DEBUG
					printf("%s", eventfile[count]);
					printf("\n");
				#endif
				}
			}
		
		} // else printf("Joystick does not exist!");
		
	}
	
	#ifdef DEBUG
		printf("lastjoy=%d\n",lastjoy);
	#endif

	for (count = 0; count <= lastjoy ; count++)
	{
	
	hasrumble[count]=0;
		
	/* Prepare the effects */
	effects[count].type = FF_RUMBLE;
	effects[count].u.rumble.strong_magnitude = 65535;
	effects[count].u.rumble.weak_magnitude = 65535;
	effects[count].replay.length = 1000;
	effects[count].replay.delay = 0;
	effects[count].id = -1;

	/* Prepare the stop event */
	stop[count].type = EV_FF;
	stop[count].code = effects[count].id;
	stop[count].value = 0;
		
	/* Prepare the play event */
	play[count].type = EV_FF;
	play[count].code = effects[count].id;
	play[count].value = 1;

		
	/* Open event file to verify if the device and the drivers support rumble */
	event_fd = open(eventfile[count], O_RDWR);
	
		
	if (event_fd < 0)
		
		/* Can't acess the event file */
		hasrumble[count]=0;
	
	else{

		if (ioctl(event_fd, EVIOCGBIT(EV_FF, sizeof(unsigned long) * 4), features) == -1)
			/* This device can't rumble, or the drivers don't support it */
			hasrumble[count]=0;
		
		else{
		
			if (test_bit(FF_RUMBLE, features))
				/* Success! This device can rumble! */
				hasrumble[count]=1;
		}
		
	/* Close the event file */
	if (event_fd>0) 	
		close(event_fd);
	
	}

	#ifdef DEBUG
		printf("%s", eventfile[count]);
	
		if (hasrumble[count]){
			printf(" can rumble.\n");
		}
		else{
			printf(" can't rumble.\n");
		}
	#endif
	
	}
	
	/* Initialization is complete */
	initialized=1;
}



/* This will run internally, as a thread */
void *joyrumble_internal(void *arg)
{

	/* Lock our thread to make it safe for multithreading */
	mtlock=1;

	int joy=int_joynumber-1;
	int event_fd;
	
	#ifdef DEBUG
		printf("1\n");
	#endif
	
	#ifdef DEBUG
		printf("%s", eventfile[joy]);
		printf("\n");
		printf("Joy#:%d\n",joy);
		printf("Strong:%d\n",int_strong[joy]);
		printf("Weak:%d\n",int_weak[joy]);
		printf("Duration:%d\n",int_duration[joy]);
	#endif
	
	/* Open the event file */
	event_fd = open(eventfile[joy], O_RDWR);


	/* Stop the effect if it's playing */
	stop[joy].code =  effects[joy].id;
	
	if (write(event_fd, (const void*) &stop[joy], sizeof(stop[joy])) == -1){
	#ifdef DEBUG
		printf("error stopping effect");
	#endif
	}
	
	
	/* Modify effect data to create a new effect */
	effects[joy].u.rumble.strong_magnitude = int_strong[joy]*65535/100;
	effects[joy].u.rumble.weak_magnitude = int_weak[joy]*65535/100;
	effects[joy].replay.length = int_duration[joy];
	effects[joy].id = -1;	/* ID must be set to -1 for every new effect */

	/* Send the effect to the driver */
	if (ioctl(event_fd, EVIOCSFF, &effects[joy]) == -1) {
		#ifdef DEBUG
			fprintf(stderr, "%s: failed to upload effect: %d\n", 
				eventfile[joy], strerror(errno));
		#endif
	}
	
	/* Play the effect */
	play[joy].code = effects[joy].id;
	
	if (write(event_fd, (const void*) &play[joy], sizeof(play[joy])) == -1){
		#ifdef DEBUG
			printf("error playing effect");
		#endif
	}
	
	/* It should be safe to unlock our thread now */
	mtlock=0;
	
	/* Sleep until the effect finishes playing */
	/* Don't worry. This is done in a separate thread and will not pause your program. */
	usleep((int_duration[joy]+50)*1000);

	#ifdef DEBUG
		printf("Done.\n");
	#endif

	/* All done. Close the event file. */
	close(event_fd);

}


/* joyrumble ( joystick number (1-8), strong motor intensity (0-100), weak motor intensity (0-100), duration in miliseconds ) */
/* This is the only function that needs to be visible externally. */
/* Notice the joystick number starts from 1. */
/* 1=/dev/input/js0 , 2=/dev/input/js1 , and so on. */
extern int joyrumble(int joynumber, int strong, int weak, int duration)
{
	
	int joy=joynumber-1;
	
	/* Copy the arguments to the shared variables, so our thread can access them. */
	int_strong[joy]=strong;
	int_weak[joy]=weak;
	int_duration[joy]=duration;

	
	#ifdef DEBUG
		printf("init=%d\n",initialized);
	#endif
	
	/* Check if the program is initialized */
	if (initialized==0) {
		#ifdef DEBUG
			printf("go init!\n");
		#endif
		joyrumble_init();
	}

	
	/* If this device doesn't support rumble, just quit */
	if (hasrumble[joy]==0){
		#ifdef DEBUG
			printf("NO RUMBLE");
		#endif
		return -1;
	}
	
	/* Sleep while our thread is locked */
	do {
		usleep(1000);
	} while (mtlock!=0);

	int_joynumber=joy+1;
	
	/* Launch the thread */
	pthread_t tid;
	if ( pthread_create( &tid, NULL, joyrumble_internal, NULL) != 0 ){
		#ifdef DEBUG
			printf("error: thread not created\n");
		#endif
		return -2;
	}
	
	
}

/* */

#define MAX_ANG 1.8

using namespace std;
using namespace ros;

Subscriber gradient_sub, vel_sub;
Publisher vel_pub;

double new_velocity, angular, max_scalar;
double near_cells_scalar[20], near_cells_theta[20];


void vectorialSum(double firstVector_m, double firstVector_a, double secondVector_m, double secondVector_a, double* resultantVector_m, double* resultantVector_a) {
	double firstVector_x = firstVector_m * cos(firstVector_a);
	double firstVector_y = firstVector_m * sin(firstVector_a);
	double secondVector_x = secondVector_m * cos(secondVector_a);
	double secondVector_y = secondVector_m * sin(secondVector_a);
	
	double resultantVector_x = firstVector_x + secondVector_x;
	double resultantVector_y = firstVector_y + secondVector_y;
	
	*resultantVector_m = sqrt(pow(resultantVector_x, 2.0) + pow(resultantVector_y, 2.0));
	*resultantVector_a = (double)((int)(firstVector_a + ((secondVector_a - firstVector_a) / 2)) % 360);
	
	if(resultantVector_m < 0){
		*resultantVector_m = -*resultantVector_m;
		*resultantVector_a = (double)((int)(*resultantVector_a + 180) % 360);
	}
}

void rumble(int level) {
	
	if (atoi(joy_dev_.c_str()) == 0) return;  
	
	if (level == 1) // soft
		joyrumble (atoi(joy_dev_.c_str()), 10, 20, 1000);
	else if (level == 2) // hard
		joyrumble (atoi(joy_dev_.c_str()), 10, 50, 1000);
	
}
	
void velCallback(const geometry_msgs::Twist::ConstPtr& msg) {
		geometry_msgs::Twist twist;
		twist.linear.x = msg->linear.x;
		twist.linear.y = msg->linear.y;
		twist.linear.z = msg->linear.z;
		twist.angular.x = msg->angular.x; 
		twist.angular.y = msg->angular.y;
		twist.angular.z = msg->angular.z; 
		if(msg->linear.x == 0) {
			twist.linear.x = 0;
			//twist.angular.z = 0;
		}
		else {
			
			if (max_scalar <= 0.5) rumble(2);
			else if (max_scalar <= 4.5) rumble(1);
			twist.linear.x = new_velocity;
			twist.angular.z = msg->angular.z+angular;
			//ROS_INFO("Actual velocity: %.2f", twist.linear.x);
		}
		
		vel_pub.publish(twist);
}

bool decel = false;

void gradientCallback(const gradient_map::GradientMap::ConstPtr& msg) {
        int size_x = msg->size_x, size_y = msg->size_y;
        std::vector<double> map_scalar = msg->map_scalar;
        std::vector<double> map_theta = msg->map_theta;
        
        near_cells_scalar[0] = map_scalar[(size_y/2 - 3)*size_x + (size_y/2 - 3)]; near_cells_theta[0] = map_theta[(size_y/2 - 3)*size_x + (size_y/2 - 3)];
        near_cells_scalar[1] = map_scalar[(size_y/2 - 3)*size_x + (size_y/2 - 2)]; near_cells_theta[1] = map_theta[(size_y/2 - 3)*size_x + (size_y/2 - 2)];
        near_cells_scalar[2] = map_scalar[(size_y/2 - 3)*size_x + (size_y/2 - 1)]; near_cells_theta[2] = map_theta[(size_y/2 - 3)*size_x + (size_y/2 - 1)];
        near_cells_scalar[3] = map_scalar[(size_y/2 - 3)*size_x + (size_y/2)];     near_cells_theta[3] = map_theta[(size_y/2 - 3)*size_x + (size_y/2)];
        near_cells_scalar[4] = map_scalar[(size_y/2 - 3)*size_x + (size_y/2 + 1)]; near_cells_theta[4] = map_theta[(size_y/2 - 3)*size_x + (size_y/2 + 1)];
        near_cells_scalar[5] = map_scalar[(size_y/2 - 3)*size_x + (size_y/2 + 2)]; near_cells_theta[5] = map_theta[(size_y/2 - 3)*size_x + (size_y/2 + 2)];
        
        /*
        near_cells[6] = map[(size_y/2 - 2)*size_x + (size_y/2 + 2)];
        near_cells[7] = map[(size_y/2 - 1)*size_x + (size_y/2 + 2)];
        near_cells[8] = map[(size_y/2)*size_x + (size_y/2 + 2)];
        near_cells[9] = map[(size_y/2 + 1)*size_x + (size_y/2 + 2)];
        near_cells[10] = map[(size_y/2 + 2)*size_x + (size_y/2 + 2)];
        
        near_cells[11] = map[(size_y/2 + 2)*size_x + (size_y/2 + 1)];
        near_cells[12] = map[(size_y/2 + 2)*size_x + (size_y/2)];
        near_cells[13] = map[(size_y/2 + 2)*size_x + (size_y/2 - 1)];
        near_cells[14] = map[(size_y/2 + 2)*size_x + (size_y/2 - 2)];
        near_cells[15] = map[(size_y/2 + 2)*size_x + (size_y/2 - 3)];
        
        near_cells[16] = map[(size_y/2 + 1)*size_x + (size_y/2 - 3)];
        near_cells[17] = map[(size_y/2)*size_x + (size_y/2 - 3)];
        near_cells[18] = map[(size_y/2 - 1)*size_x + (size_y/2 - 3)];
        near_cells[19] = map[(size_y/2 - 2)*size_x + (size_y/2 - 3)];
        */
        
        //Find the maximum repulsive force
        max_scalar = near_cells_scalar[0];
        double max_theta = near_cells_theta[0];
        /*for(int i=1; i<6; i++){
			if(near_cells_scalar[i] > max_scalar){
				max_scalar = near_cells_scalar[i];
				max_theta = near_cells_theta[i];
			}
		}*/
		
		for (int i = 1; i < 6; i++)
			vectorialSum(max_scalar, max_theta, near_cells_scalar[i], near_cells_theta[i], &max_scalar, &max_theta);
			
		max_scalar = 0;
		for (int i = 0; i < 6; i++)
			max_scalar+=near_cells_scalar[i];
		
		max_scalar/=6;
		
		if (max_scalar <= 6) {
			if ((max_theta > 247.5 && max_theta <= 292.5) || (max_theta > 67.5 && max_theta <= 112.5))
				angular = 0;
			else if ((max_theta > 292.5 && max_theta <= 337.5) || (max_theta > 22.5 && max_theta <= 67.5))
				angular = -MAX_ANG/2;
			else if (max_theta > 337.5 || max_theta <= 22.5)
				angular = -MAX_ANG;
			else if ((max_theta > 112.5 && max_theta <= 157.5) || (max_theta > 202.5 && max_theta <= 247.5))
				angular = MAX_ANG/2;
			else if (max_theta > 157.5 && max_theta <= 202.5)
				angular = MAX_ANG;
		}
		else angular = 0;
		//ROS_INFO("scalar: %.2f, angle: %.2f, angular: %.2f", max_scalar, max_theta, angular);
		
		
		//Calculate the smoothed deceleration
		if(max_scalar == 1000.0)
			new_velocity = 0.0;
		else if(max_scalar < 12){	//further than 8 steps there's no deceleration
			
			if(max_scalar <= 2.5) new_velocity = 0.0; //if we are close to the obstacle, there's no movement on the X axis
			else if(max_scalar <= 3.5) new_velocity = 0.1;
			else if(max_scalar <= 4.5) new_velocity = 0.3;
			else if(max_scalar <= 5.5) new_velocity = 0.4;
			else if(max_scalar <= 6.5) new_velocity = 0.6; // 1.4
			else if(max_scalar <= 7.0) new_velocity = 0.8; // 1.8
			else new_velocity = 1.1;
			
			decel = true;
			
		} else {
			
			if (decel) {
				new_velocity += 0.1;
				if (new_velocity > 1.4) decel = false;
			}
			else new_velocity = 1.5;
			
		}
}

int main(int argc, char **argv) {
	init(argc, argv, "safe_teleop");
	NodeHandle n;
	gradient_sub = n.subscribe("gradient_map", 1, gradientCallback);
	vel_sub = n.subscribe("vel", 1, velCallback); 
	vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	ros::NodeHandle nh_param("~");
	nh_param.param<std::string>("dev", joy_dev_, "/dev/input/js1");

	//ROS_INFO("Joystick is %s", joy_dev_.c_str());
	//rumble(1);
	
    spin(); 

        return 0;
}
