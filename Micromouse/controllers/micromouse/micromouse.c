/* for navigation */
#include <webots/compass.h>
#include <string.h>
#include <stdlib.h>
#include <webots/gps.h>
#include <webots/pen.h>
/* for wall follow */
#include <webots/robot.h>
#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
#include <webots/light_sensor.h>
#include <webots/camera.h>
#include <webots/accelerometer.h>
#include <webots/led.h>
#include <stdio.h>
#include <math.h>
/* for navigation */
#define TIME_STEP 64
#define WHEEL_RADIUS 0.0205
#define AXLE_LENGTH 0.052
#define ENCODER_RESOLUTION 159.23
#define RANGE (1024 / 2)
#define TRUE 1
#define MAX_COMPASS_VALUE 180
/* for wall follow */
#define TIME_STEP 64
#define WHEEL_RADIUS 0.0205
#define AXLE_LENGTH 0.052
#define ENCODER_RESOLUTION 159.23
#define RANGE (1024 / 2)
#define FRONT_OBSTACLE_THRESHOLD 200
#define LOW_FRONT_OBSTACLE_THRESHOLD 100
#define HIGH_WALL_DISTANCE_THRESHOLD 300
#define LOW_WALL_DISTANCE_THRESHOLD 200
#define ONE 1
#define ZERO 0
#define ALL_SENSORS 8
//compass values are converted to degrees in range of -180 to +180
/*verbose options*/
static int verbose = 0;  //zero for no stats
//verbose = -1;         //-1 for bearing stats
//verbose = 1;         //targets and current stats
int at_mline=0, obstacle=0;
/* destination point */
/* enter waypoint[3] in form {Z, Y, X} */
double waypoint[3] = {0.2,0,-0.3};
//double waypoint[3] = {-0.02,0,0.018};
/* set m-line */
double reference_angle = 0;
double current_angle = 0;
enum State_type {
  GO_STRAIGHT, GO_RIGHT, GO_LEFT, SPIN_RIGHT, SPIN_LEFT
} state;
/* sensor variables */
WbDeviceTag distance_sensor[8];
int i;
double speed[2];
double sensors_value[8];
typedef int boolean;
double target_bearing;
double left_wheel_speed, right_wheel_speed;
enum Found_notfound {
  NOT_FOUND, FOUND
} found_obstacle;
double eb = 0, last_eb = 0, cumulative_eb = 0, change_eb = 0;
double current_bearing=0, error_bearing=0, last_error_bearing=0, 
       cumulative_error_bearing=0, change_error_bearing=0;
double dt = 1; // duration of one time step for pid output calculation
double Kp = 20.0, Ki = .001, Kd =0.0001; // pid constants
double output = 0; // pid output
double epsilon = 0.5; // max error for convergence
int time_step = 0;
double epsilon2 = 0.05;
double c_output = 0;
// eb = last_eb = cumulative_eb = 0;
static int wildcard = 1;
double compass_noise;
static int avoidObstacle_isactive= 1;
static double saved_euclidian= 0;
// error_bearing = last_error_bearing = cumulative_error_bearing = 0;
/* prototypes */
int avoidObstacle();
int navigate(const double *,WbDeviceTag,WbDeviceTag,const double*);
void check_status_of_epuck(int*,int*);
/* get angle in degrees using compass */
double 
get_current_bearing_in_degrees(WbDeviceTag tag) 
{
  const double *north = wb_compass_get_values(tag);
  double rad = atan2(north[0],north[2]);
  //double bearing = rad * (360/M_PI);
  double bearing = (rad - 1.5708) / M_PI * 180.0;

  if (bearing < 0.0)
    bearing=360+bearing;
  //printf("BEARING %.2f\n", bearing);

  return (bearing-180) * -1.0;
}
/* get angle in degrees using slope */
double 
w_get_current_bearing_in_degrees(double *wp, const double *coor) 
{
  //const double *north = wb_compass_get_values(tag);
  double rad = atan2((wp[0]-coor[2]),(wp[2]-coor[0]));
  //double bearing = rad * (360/M_PI);
  double bearing = (rad - 1.5708) / M_PI * 180.0;

  if (bearing < 0.0)
    bearing=360+bearing;
  //printf("BEARING %.2f\n", bearing);

  return (bearing-180) * -1.0;
}
/* get out of avoid obstacle */
void
check_status_of_epuck(int *m, int *obj)
{
  int lastm=*m;
  int lastobj=*obj;
  *m = ((int)(current_angle))<(((int)(reference_angle)+6))
    && ((int)(current_angle))>(((int)(reference_angle)-6));
  if(avoidObstacle_isactive)
    *obj= avoidObstacle();
  else
    *obj= 0;
  if(lastm==0 && lastobj==1 && *m==1 ) {
    *obj= 0;
    state = GO_STRAIGHT;
  }
}
int 
main()
{  
  wb_robot_init();
  /* compass tag */
  WbDeviceTag compass = wb_robot_get_device("compass");
  wb_compass_enable(compass,TIME_STEP);
  /* gps tag */
  WbDeviceTag gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, 10);
  /* pen tag */
  WbDeviceTag pen = wb_robot_get_device("pen");
  wb_pen_write(pen, 1);
  /* set m-line */
  const double * start_point_coordinates = (wb_gps_get_values(gps));
  reference_angle = w_get_current_bearing_in_degrees(
    waypoint, start_point_coordinates);
  /* name sensors */
  for (i = 0; i < 8; i++) {
    char device_name[4];
    /* get distance sensors */
    sprintf(device_name, "ps%d", i);
    distance_sensor[i] = wb_robot_get_device(device_name);
    wb_distance_sensor_enable(distance_sensor[i],TIME_STEP*4);
  }
  /* default motion for avoidObstacle() */
  state = GO_STRAIGHT;
  found_obstacle = NOT_FOUND;
  const double * coordinates;
  /* navigate and avoidObstacle */
  /* main loop */
  while(true) 
  { 
    /* get current coordinates */
    coordinates = (wb_gps_get_values(gps));
    /* get current angle */
    current_angle = w_get_current_bearing_in_degrees(waypoint, coordinates);
    check_status_of_epuck(&at_mline,&obstacle);
    /* if we are at m-line and there is no obstacle */
    if((at_mline && !obstacle)
    ||(!at_mline && !obstacle)) 
    {
      if(!navigate(coordinates, gps, compass,start_point_coordinates))
        break;
      if(verbose) puts("in navigate");
    }
    else 
      if((!at_mline && obstacle)
      ||  (at_mline && obstacle)
      ) 
    {
      if(avoidObstacle_isactive) avoidObstacle();
      if(verbose) puts("in avoid");
    }

    if(verbose) {
        printf("current_angle= %d, reference_angle= %d\n", 
        (int)(current_angle), (int)(reference_angle));
        printf("m= %d, obj= %d\n", at_mline, obstacle);
    }
  }
  return ZERO;
}
int 
avoidObstacle() {  
  /* if it finds obstacle we return 1 else 0 for not found obstacle*/
  found_obstacle = FOUND;
  int number_negligible_sensoreadings = 0;
  /* initialize sensors */
  for (i = 0; i < 8; i++) {
    sensors_value[i] = wb_distance_sensor_get_value(distance_sensor[i]);
    if((int)sensors_value[i]<60)
      number_negligible_sensoreadings++;
  }
  /* if low sensor readings, NOT object FOUND */
  if(number_negligible_sensoreadings==ALL_SENSORS)
    return NOT_FOUND;
  // no obstacles ahead, keep going straight
  if (state == GO_STRAIGHT) {
    if ((sensors_value[0] < FRONT_OBSTACLE_THRESHOLD)
     && (sensors_value[7] < FRONT_OBSTACLE_THRESHOLD))
    {
      speed[0] = speed[1] = 200; 
      found_obstacle = NOT_FOUND;
    }
    else if (sensors_value[7] > FRONT_OBSTACLE_THRESHOLD) {
      state = SPIN_RIGHT;
    }
    else {
      state = SPIN_LEFT;
    }
  }
  else 
  if (state == SPIN_RIGHT){
    // front left sensor picked obstacle, spin right until front left sensor 
    // does not pick up something very near
    if (fabs(sensors_value[7]) > LOW_FRONT_OBSTACLE_THRESHOLD
    ||      (sensors_value[6] > FRONT_OBSTACLE_THRESHOLD))
    {
      speed[0] = 200;
      speed[1] = -200;
    }
    else {
      if(verbose) puts("GO_RIGHT");
      state = GO_RIGHT;
    }
  }   
  else if (state == SPIN_LEFT){
    // front right sensor picked obstacle, spin left until front right sensor 
    // does not pick up something very near
    if ((sensors_value[7] > FRONT_OBSTACLE_THRESHOLD) 
     || (sensors_value[6] > LOW_FRONT_OBSTACLE_THRESHOLD)){
      speed[0] = -200;
      speed[1] = 200;
    }
    else {
      if(verbose) puts("GO_RIGHT");
      state = GO_RIGHT;
    }
  }
  else if (state == GO_RIGHT){
    double s6s5_diff = sensors_value[5] - sensors_value[6];
    //if(verbose) printf("S1, S2, diff = %.2f, %.2f, %.2f\n", sensors_value[1],sensors_value[2], s6s5_diff);
    if(verbose) printf("s6s5_diff: %d\n", (int)s6s5_diff );
    if ((sensors_value[7] > FRONT_OBSTACLE_THRESHOLD) || 
        (sensors_value[0] > LOW_FRONT_OBSTACLE_THRESHOLD) || 
        (sensors_value[1] > LOW_FRONT_OBSTACLE_THRESHOLD))
    {
    // obstacle ahead - spin right to get wall on left
      state = SPIN_RIGHT;
      if(verbose) puts("SPIN_RIGHT");
    }
    else if (s6s5_diff > HIGH_WALL_DISTANCE_THRESHOLD){
      // get close to wall - turn slight left
      speed[0] = 200 + (sensors_value[6] - 200);
      speed[1] = 175;
      if(verbose) puts("Turning slight left");
      //if(verbose) printf("left wheel speed: %d\n", (int)speed[0]);
    }
    else if ((s6s5_diff > 0) 
          && (s6s5_diff < HIGH_WALL_DISTANCE_THRESHOLD))
    {
      // get far from wall - turn slight right
      speed[0] = 175 + (sensors_value[6] - 200);
      speed[1] = 200;
      if(verbose) puts("Turning slight right");
    }
    else if (s6s5_diff < 0) {
      // robot has turned too much to the right, possibly after turning 
      // a corner while doing follow wall right -- spin back left on axis
      state = SPIN_RIGHT;
      if(verbose) puts("SPIN_RIGHT");
    }
    else if ( (sensors_value[5] < 100) 
           && (sensors_value[6] < 100)){
    // lost the wall while following it -- turn left
      speed[0] = 200;
      speed[1] = 250;
      found_obstacle = NOT_FOUND;
    }

    else { // robot at correct distance threshold - go straight
      speed[0] = speed[1] = 200;
      found_obstacle = NOT_FOUND;
      //if(verbose) printf("Following wall\n");
    }
  }
  /* set speed values */
  //if(verbose) printf("State = %d\n", state);
  wb_differential_wheels_set_speed(speed[0]*1.5,speed[1]*1.5);
  //wb_differential_wheels_set_speed(250,250);
  /* perform a simulation step */
  wb_robot_step(TIME_STEP);
  return found_obstacle;
}
double 
difference(double currentBearings,double destinationBearings)
{
  double diff= destinationBearings - currentBearings;
  if (diff>180.0)
  {
    diff=diff-360.0;

  }
  if(diff<-180.0)
  {
    diff=diff+360.0;
  }
  return diff;
}
int 
navigate(const double * curdinate, WbDeviceTag gps,
         WbDeviceTag compass, const double * spc) 
{
  time_step++;      
  //get current bearing and add random uniform noise in -0.5 to 0.5
  compass_noise = 0;
  //printf("Compass noise = %.2f", compass_noise);
  current_bearing = get_current_bearing_in_degrees(compass)
   + compass_noise; 

  // const double * coordinates = (wb_gps_get_values(gps));

  // double slope_angle = w_get_current_bearing_in_degrees(waypoint, curdinate);
  // target_bearing = slope_angle;
  target_bearing = current_angle+180;
  // calculate the terms for the pid equation
  last_error_bearing = error_bearing;   // differential term
  error_bearing = difference (current_bearing, target_bearing); //error term
  change_error_bearing= error_bearing - last_error_bearing;
  if (fabs(cumulative_error_bearing) < MAX_COMPASS_VALUE)
    cumulative_error_bearing += error_bearing;  //integral term, only add if error > epsilon_error

  // the pid equation
  output = Kp * error_bearing + (Ki * cumulative_error_bearing * dt) + (Kd * change_error_bearing /dt);
  output = fabs(output);

  double euclidean_d;
  euclidean_d =
  sqrt(pow((waypoint[0])-(curdinate[2]),2) + 
       pow((waypoint[2])-(curdinate[0]),2));
  if(saved_euclidian==0)
    saved_euclidian = euclidean_d;

  //calculate terms for PID equation when moving to coordinates
  last_eb = eb;
  eb = euclidean_d; 
  change_eb = eb - last_eb;
  if( cumulative_eb < euclidean_d )
    cumulative_eb += eb;
  c_output = (Kp * eb + (Ki * cumulative_eb * dt) + (Kd * change_eb /dt));
  // c_output *= -1;
  if(verbose==1){
  printf("\n\nTarget coordinates \t %.2f, %.2f\n", waypoint[0],waypoint[2]);
  printf("Dynamic coordinates\t %.2f, %.2f\n", curdinate[0], curdinate[2]);
  printf("Current Euclidean\t\t %.2f\n\n", euclidean_d);
  printf("Target angle\t\t\t %.2f\n", current_angle);
  printf("Current angle\t\t\t %.2f\n", current_bearing);
  printf("Velocity (c_output)\t %.2f\n", -1*c_output);
  printf("Angular velocity\t\t %.2f\n\n", output);
  }

  //verbose= -1;
  if(verbose==-1)
  printf (
  "At step: %d: current bearing = %.2f, error_bearing = %.2f, cumulative_error_bearing = %.2f, control output = %.2f\n",
  time_step, current_bearing, error_bearing, cumulative_error_bearing, output);

  if(saved_euclidian>euclidean_d)
    avoidObstacle_isactive= 0;

  if ((output > epsilon))
  {
    left_wheel_speed = -output;
    right_wheel_speed = output;
    if(wildcard) reference_angle = 
      w_get_current_bearing_in_degrees(waypoint, spc);
  }
  else
    if ((c_output) > epsilon2){
      saved_euclidian = euclidean_d;
      avoidObstacle_isactive= 1;
      wildcard = 0;
      left_wheel_speed = right_wheel_speed = c_output*100;    
  }

  if(verbose) printf("%.5f, %.5f\n", saved_euclidian, euclidean_d );
  /* Test values
  printf("\ncoordinates %d, %d\n",
  (int)(coordinates[0]*10),(int)(coordinates[2]*10));
  printf("vs waypoint %d, %d\n\n",
  (int)(waypoint[2]*10),(int)(waypoint[0]*10));
  */

  if(
  (
  (int)(curdinate[0]*10)==(int)(waypoint[2]*10) 
  && (int)(curdinate[2]*10)==(int)(waypoint[0]*10)
  )
  || (int)(euclidean_d*100) == 0)
  {
    left_wheel_speed = right_wheel_speed = 0;
    return ZERO;
  }

  wb_differential_wheels_set_speed(left_wheel_speed,right_wheel_speed);
  wb_robot_step(TIME_STEP);

  return ONE;
}
