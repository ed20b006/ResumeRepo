#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include "turtlesim/Spawn.h"
#include "iostream"
#include "cppad/ipopt/solve.hpp"
#define PI 3.14159265

turtlesim::Pose turtle1, turtle2;
turtlesim::Pose startPoint;
turtlesim::Pose endPoint;
float minDis;

bool calculator();
void spawn_turtle2(ros::NodeHandle);
void get_turtle1_pose(const turtlesim::Pose::ConstPtr);
void get_turtle2_pose(const turtlesim::Pose::ConstPtr);
void prediction_horizon();
void control_action();
float line_point_dist(turtlesim::Pose);
turtlesim::Pose position_calculator(turtlesim::Pose,float);

float line_point_dist(turtlesim::Pose t)
{
  double A, B, C, d;
  A = startPoint.y - endPoint.y;
  B = endPoint.x - endPoint.x;
  C = endPoint.y * startPoint.x - endPoint.x * startPoint.y;
  d = pow(A, 2) + pow(B, 2);
  return ((A * t.x + B * t.y + C) / d);
}

turtlesim::Pose position_calculator(turtlesim::Pose, float);
namespace
{
  using CppAD::AD;
  using CppAD::Value;

  class FG_eval
  {
  public:
    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
    void operator()(ADvector &fg, const ADvector &x)
    {
      assert(fg.size() == 5);
      assert(x.size() == 8);

      // Fortran style indexing
      //1111
      AD<double> w[4];
      AD<double> v[4];
      AD<double> posx[4];
      AD<double> posy[4];
      AD<double> theta[4];
      posx[0] = turtle2.x;
      posy[0] = turtle2.y;
      theta[0] = turtle2.theta;
      v[0] =x[0];
      w[0] =x[1];
      fg[0] = pow(line_point_dist(turtle2),2) + pow((turtle2.x-turtle1.x),2) + pow((turtle2.y-turtle1.y),2);
      fg[1] = turtle2.linear_velocity;
      turtlesim::Pose prevPos,nextPos;
      for(int i=1;i<4;i++)
      {
        prevPos.x = Value(posx[i-1]);
        prevPos.y = Value(posy[i-1]);
        prevPos.theta = Value(theta[i-1]);
        prevPos.linear_velocity = Value(v[i-1]);
        prevPos.angular_velocity = Value(w[i-1]);
        nextPos = position_calculator(prevPos,1);
        posx[i] = nextPos.x;
        posy[i] = nextPos.y;
        theta[i] = nextPos.theta;
        v[i] = x[i*2];
        w[i] = x[i*2+1];
        fg[0] += pow(line_point_dist(nextPos),2) + pow((nextPos.x-turtle1.x),2) + pow((nextPos.y-turtle1.y),2);
        fg[0] += pow(nextPos.angular_velocity,2);
        fg[i] = nextPos.linear_velocity;
      }

      // f(x)
      // g_1 (x)
      //fg[1] = x1 * x2 * x3 * x4;
      // g_2 (x)
      //fg[2] = x1 * x1 + x2 * x2 + x3 * x3 + x4 * x4;
      //
      return;
    }
  };
}

bool calculator(void)
{
  bool ok = true;
  size_t i;

  typedef CPPAD_TESTVECTOR(double) Dvector;

  // number of independent variables (domain dimension for f and g)
  size_t nx = 4;
  // number of constraints (range dimension for g)
  size_t ng = 2;
  // initial value of the independent variables
  Dvector xi(nx);
  xi[0] = 1.0;
  xi[1] = 5.0;
  xi[2] = 5.0;
  xi[3] = 1.0;
  // lower and upper limits for x
  Dvector xl(nx), xu(nx);
  for (i = 0; i < nx; i++)
  {
    xl[i] = 1.0;
    xu[i] = 5.0;
  }
  // lower and upper limits for g
  Dvector gl(ng), gu(ng);
  gl[0] = 25.0;
  gu[0] = 1.0e19;
  gl[1] = 40.0;
  gu[1] = 40.0;

  // object that computes objective and constraints
  FG_eval fg_eval;

  // options
  std::string options;
  // turn off any printing
  options += "Integer print_level  0\n";
  options += "String  sb           yes\n";
  // maximum number of iterations
  options += "Integer max_iter     10\n";
  // approximate accuracy in first order necessary conditions;
  // see Mathematical Programming, Volume 106, Number 1,
  // Pages 25-57, Equation (6)
  options += "Numeric tol          1e-6\n";
  // derivative testing
  options += "String  derivative_test            second-order\n";
  // maximum amount of random pertubation; e.g.,
  // when evaluation finite diff
  options += "Numeric point_perturbation_radius  0.\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, xi, xl, xu, gl, gu, fg_eval, solution);
  //
  // Check some of the solution values
  //
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;
  //
  /*
     double check_x[]  = { 1.000000, 4.743000, 3.82115, 1.379408 };
     double check_zl[] = { 1.087871, 0.,       0.,      0.       };
     double check_zu[] = { 0.,       0.,       0.,      0.       };
     double rel_tol    = 1e-6;  // relative tolerance
     double abs_tol    = 1e-6;  // absolute tolerance
     for(i = 0; i < nx; i++)
     {     ok &= CppAD::NearEqual(
               check_x[i],  solution.x[i],   rel_tol, abs_tol
          );
          ok &= CppAD::NearEqual(
               check_zl[i], solution.zl[i], rel_tol, abs_tol
          );
          ok &= CppAD::NearEqual(
               check_zu[i], solution.zu[i], rel_tol, abs_tol
          );
     }*/
  //std::cout << solution.x;
  return ok;
}

turtlesim::Pose position_calculator(turtlesim::Pose prevPos, float time)
{
  turtlesim::Pose nextPos;
  if (prevPos.angular_velocity != 0)
  {
    nextPos.x = (prevPos.linear_velocity / prevPos.angular_velocity) * sin(prevPos.angular_velocity * time + prevPos.theta);
    nextPos.y = (prevPos.linear_velocity / prevPos.angular_velocity) * cos(prevPos.angular_velocity * time + prevPos.theta);
  }
  else
  {
    nextPos.x = prevPos.linear_velocity * time;
    nextPos.y = prevPos.linear_velocity * time;
  }

  return nextPos;
}

void spawn_turtle2(ros::NodeHandle n)
{
  ros::ServiceClient client = n.serviceClient<turtlesim::Spawn>("spawn");

  turtlesim::Spawn spawn;
  spawn.request.x = startPoint.x;
  spawn.request.y = startPoint.y;
  if ((endPoint.x - startPoint.x) == 0)
  {
    if ((endPoint.y - startPoint.y) > 0)
      spawn.request.theta = PI;
    else
      spawn.request.theta = -PI;
  }
  else
  {
    spawn.request.theta = atan((endPoint.y - startPoint.y) / (endPoint.x - startPoint.x));
  }

  spawn.request.name = "turtle2";
  if (client.call(spawn))
  {
    ROS_INFO("Spawned");
  }
  else
  {
    ROS_ERROR("Failed to call service spawn");
  }
}

void get_turtle1_pose(const turtlesim::Pose::ConstPtr &pos)
{
  turtle1 = *pos;
}

void get_turtle2_pose(const turtlesim::Pose::ConstPtr &pos)
{
  turtle2 = *pos;
}

int main(int argc, char **argv)
{
  float dis = 0, velocity = 0;

  std::cout << "Enter min distance : "; // getting min distance for overtaking
  std::cin >> minDis;
  std::cout << "Velocity : "; //velocity of turtle
  std::cin >> velocity;
  std::cout << "Enter starting point : ";
  std::cout << "\nx:";
  std::cin >> startPoint.x;
  std::cout << "y:";
  std::cin >> startPoint.y;
  std::cout << "Enter end point : ";
  std::cout << "\nx:";
  std::cin >> endPoint.x;
  std::cout << "y:";
  std::cin >> endPoint.y;

  ros::init(argc, argv, "Overtake");

  ros::NodeHandle n;

  spawn_turtle2(n);
  //creating variables for publishing velocity of turtle
  ros::Publisher turtle2_vel_pub = n.advertise<geometry_msgs::Twist>("/turtle2/cmd_vel", 100);

  //creating variables for subscribing to get turtle 1 & 2 postion
  ros::Subscriber turtel1_pose_sub = n.subscribe<turtlesim::Pose>("/turtle1/pose", 100, get_turtle1_pose);
  ros::Subscriber turtel2_pose_sub = n.subscribe<turtlesim::Pose>("/turtle2/pose", 100, get_turtle2_pose);

  geometry_msgs::Twist vel;

  vel.linear.x = 2.0;
  vel.linear.y = 0.0;
  vel.linear.z = 0.0;
  vel.angular.x = 0.0;
  vel.angular.y = 0.0;
  vel.angular.z = 0.0;

  turtle2_vel_pub.publish(vel);

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    if (pow(turtle2.x - endPoint.x, 2) + pow(turtle2.y - endPoint.y, 2) < 0.04)
    {
      vel.linear.x = 0.0;
      vel.linear.y = 0.0;
      vel.linear.z = 0.0;
      vel.angular.x = 0.0;
      vel.angular.y = 0.0;
      vel.angular.z = 0.0;

      turtle2_vel_pub.publish(vel);
      break;
    }

    ROS_INFO("[%f] [%f]\n", turtle2.x, turtle2.y);
    calculator();

    turtle2_vel_pub.publish(vel);

    ros::spinOnce();

    loop_rate.sleep();
  }
}
