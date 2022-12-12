#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <iostream>
#include <fstream>
#include <string>
#include <math.h>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;
//Eigen 네임 스페이스 선언

mavros_msgs::State current_state;
geometry_msgs::PoseStamped current_pose;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pose = *msg;
}
//const ptr&msg subscribe 토픽(의 형태->msg를 전송), bind 통해 묶어준 것

float dist(geometry_msgs::PoseStamped curr, mavros_msgs::PositionTarget des){
    return pow((curr.pose.position.x - des.position.x), 2) + pow((curr.pose.position.y - des.position.y),2) + pow((curr.pose.position.z - des.position.z),2) ;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");     //노드 이름 선언
    ros::NodeHandle nh;     //하나의 노드에 있는 액세스 포인트, 노드 간 신호
    ros::Rate rate(3);

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, pos_cb);
    ros::Publisher local_pos_sp_pub = nh.advertise<mavros_msgs::PositionTarget>
            ("mavros/setpoint_raw/local", 10);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();    //subscribe한 거 확인하고 없으면 다음 것으로 topic에 callback 실행
        rate.sleep();   //시간에 맞춰 한 번 돌도록 하는 코드
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    mavros_msgs::PositionTarget des_vel;
    Eigen::Vector3d ang_vel(0, 0, 1);   //Eigen에 ang_vel변수 선언
    Eigen::Vector3d curr_pos;   //Eigen에 curr_pos변수 선언
    Eigen::Vector3d ref_vel;    //Eigen에 ref_vel변수 선언
    bool ready = false;
    des_vel.position.x = 0.0f;      //x좌표 선언
    des_vel.position.y = 0.0f;      //y좌표 선언
    des_vel.position.z = 0.0f;      //z좌표 선언
    des_vel.acceleration_or_force.x = 0.0f;     //x가속도 선언
    des_vel.acceleration_or_force.y = 0.0f;     //y가속도 선언
    des_vel.acceleration_or_force.z = 0.0f;     //Z가속도 선언
    des_vel.velocity.x = 0.0f;      //x속도 선언
    des_vel.velocity.y = 0.0f;      //y속도 선언
    des_vel.velocity.z = 0.0f;      //z속도 선언
    des_vel.yaw = 0.0f;             //hovering상태여서 yaw값만 선언
    des_vel.yaw_rate = 0.0f;
    ROS_INFO("s");
    if (local_pos_sp_pub)
    {
      ROS_INFO("zero velocity");
      for (int i = 100; ros::ok() && i > 0; --i)
      {
        local_pos_sp_pub .publish(des_vel);     //desire velocity를 퍼블리시 한다.
        ros::spinOnce();
        // rate.sleep()
        ros::Duration(0.01).sleep();
      }
      ROS_INFO("Done with zero velocity set");
    }

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");

            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");      //드론의 시동을 키는 코드

                }
                last_request = ros::Time::now();
            }
        }
        // TODO. hovering
        des_vel.position.x = 2;
        des_vel.position.y = 2;
        des_vel.position.z = 1;         //드론을 (2,2,1)로 이동시킨다.
    
        curr_pos[0]=current_pose.pose.position.x;
        curr_pos[1]=current_pose.pose.position.y;
        curr_pos[2]=current_pose.pose.position.z;
        if(ready==false){
        ROS_INFO("dist: %f",dist(current_pose,des_vel)); //현재 오차 출력한다.
        if(dist(current_pose,des_vel)<0.001){ //지정한 위치에 도달하면 다음좌표로 이동한다.
            ROS_INFO("end");
            ready=true;
        }
      }
      else{
        // TODO. calculate velocity
        ref_vel=ang_vel.cross(curr_pos);  //현재 위치과 각속도를 외적해서 바디 프레임의 속도를 구한다.
        ROS_INFO("x%f",ref_vel[0]);  //x축의 바디 프레임 속도
        ROS_INFO("y%f",ref_vel[1]);  //y축의 바디 프레임 속도
        ROS_INFO("z%f",ref_vel[2]);  //z축의 바디 프레임 속도
      }

        // TODO. set calculated velocity
      des_vel.velocity.x = ref_vel[0];
      des_vel.velocity.y = ref_vel[1];
      des_vel.velocity.z = ref_vel[2];  //계산한 바디 프레임의 속도를 desire 속도에 대입한다.


        // publish des_pos topic
        local_pos_sp_pub.publish(des_vel);  //desire 속도를 topic의 형식으로 퍼블리시한다.

        ros::spinOnce();
        rate.sleep();

    }

    return 0;
}