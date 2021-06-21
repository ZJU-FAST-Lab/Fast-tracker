#ifndef _SFC_H
#define _SFC_H
#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <Eigen/Eigen>
#include <nav_msgs/Path.h>
#include <utility>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <Eigen/Eigen>
#include <std_msgs/Float64.h>
#include <grid_path_searcher/astar.h>

class FlightCube
{
public:
  NodePtr start_node;
  NodePtr end_node;
  //           ->x_pos
  //           y_pos
  double x_pos;
  double x_neg;
  double y_pos;
  double y_neg;
  double z_pos;
  double z_neg;
  int x_pos_int;
  int x_neg_int;
  int y_pos_int;
  int y_neg_int;
  int z_pos_int;
  int z_neg_int;
  int borders_int[6];//0 for xl,1 for xu,2 for yl,3 for yu,4 for zl,5 for z
  double borders[6];



  FlightCube(NodePtr s_n,NodePtr e_n)
  {
    start_node=s_n;
    end_node=e_n;
    if(end_node->index[0]>start_node->index[0])//init x
    {
      x_pos_int=end_node->index[0]-start_node->index[0];
      x_neg_int=0;
    }
    else
    {
      x_neg_int=start_node->index[0]-end_node->index[0];
      x_pos_int=0;
    }

    if(end_node->index[1]>start_node->index[1])//init y
    {
      y_pos_int=end_node->index[1]-start_node->index[1];
      y_neg_int=0;
    }
    else
    {
      y_neg_int=start_node->index[1]-end_node->index[1];
      y_pos_int=0;
    }

    if(end_node->index[2]>start_node->index[2])//init z
    {
      z_pos_int=end_node->index[2]-start_node->index[2];
      z_neg_int=0;
    }
    else
    {
      z_neg_int=start_node->index[2]-end_node->index[2];
      z_pos_int=0;
    }
  }

  void Display()
  {
    // ROS_INFO("start_node_x_int=%d   y_int=%d   z_int=%d     x_pos_int=%d  y_pos_int=%d  z_pos_int=%d  x_neg_int=%d  y_neg_int=%d  z_neg_int=%d",
    // start_node->index[0],start_node->index[1],start_node->index[2],x_pos_int,y_pos_int,z_pos_int,x_neg_int,y_neg_int,z_neg_int);

    ROS_INFO("start_node_x=%f   y=%f  z=%f     x_pos=%f  y_pos=%f  z_pos=%f  x_neg=%f  y_neg=%f  z_neg=%f",
    start_node->position[0],start_node->position[1],start_node->position[2],x_pos,y_pos,z_pos,x_neg,y_neg,z_neg);

    ROS_INFO("end_node_x=%f   y=%f  z=%f     x_pos=%f  y_pos=%f  z_pos=%f  x_neg=%f  y_neg=%f  z_neg=%f",
    end_node->position[0],end_node->position[1],end_node->position[2],x_pos,y_pos,z_pos,x_neg,y_neg,z_neg);



    for (int i=0;i<6;i++)
      ROS_INFO("border=%f",borders[i]);
  }

};




class FlightCorridor
{
public:
  std::vector<FlightCube> cubes;
  // int max_expand_size;

  bool check_cube_safe(FlightCube cube,std::vector<Eigen::Vector3i> &collision_grids,int last_node_order,
    int check_order,int flag=0);
  bool check_cube_safe(FlightCube cube,int flag=0);
  FlightCube expand_cube(FlightCube &cube);

  void update_attributes(FlightCube &cube);


  void divide_corridor(double min_length);

  void divide_one_corridor();

  Eigen::MatrixXd corridor2mat();

  int generate(std::vector<Eigen::Vector3d> gridpath);

  void set_map(GridMap::Ptr env){gridmap = env;}
private:
  GridMap::Ptr gridmap;

};
#endif
