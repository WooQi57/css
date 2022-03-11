/************************************************* 
Copyright:Volcano Robot 
Author: 锡城筱凯
Date:2021-02-04 
Blog：https://blog.csdn.net/xiaokai1999
Description:
**************************************************/  
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <signal.h>
#include <std_msgs/String.h>

#include <webots_ros/set_float.h>
#include <webots_ros/set_int.h>
#include <webots_ros/Int32Stamped.h>
#include <webots_ros/Float64Stamped.h>

using namespace std;

#define TIME_STEP 32    //时钟
#define NMOTORS 5       //机械臂电机数量
ros::NodeHandle *n;

static int controllerCount;
static std::vector<std::string> controllerList; 

ros::ServiceClient timeStepClient;          //时钟通讯客户端
webots_ros::set_int timeStepSrv;            //时钟服务数据

ros::ServiceClient velocity_client;
webots_ros::set_int velocity_srv;

ros::ServiceClient position_client;   
webots_ros::set_int position_srv;

vector<double> armpositions = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
vector<double> armvelocities = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
static const char *armNames[NMOTORS] = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5"};

void joint_1_sensorcallback(const webots_ros::Float64Stamped::ConstPtr &value);
void joint_2_sensorcallback(const webots_ros::Float64Stamped::ConstPtr &value);
void joint_3_sensorcallback(const webots_ros::Float64Stamped::ConstPtr &value);
void joint_4_sensorcallback(const webots_ros::Float64Stamped::ConstPtr &value);
void joint_5_sensorcallback(const webots_ros::Float64Stamped::ConstPtr &value);

/*******************************************************
* Function name ：quit
* Description   ：退出函数
* Parameter     ：
        @sig   信号
* Return        ：无
**********************************************************/
void quit(int sig) {
    ROS_INFO("User stopped the '/cssgra' node.");
    timeStepSrv.request.value = 0; 
    timeStepClient.call(timeStepSrv); 
    ros::shutdown();
    exit(0);
}

/*******************************************************
* Function name ：controllerNameCallback
* Description   ：控制器名回调函数，获取当前ROS存在的机器人控制器
* Parameter     ：
        @name   控制器名
* Return        ：无
**********************************************************/
// catch names of the controllers availables on ROS network
void controllerNameCallback(const std_msgs::String::ConstPtr &name) { 
    controllerCount++; 
    controllerList.push_back(name->data);//将控制器名加入到列表中
    ROS_INFO("Controller #%d: %s.", controllerCount, controllerList.back().c_str());

}


void Joint_State_Publish(ros::Publisher pub){
    sensor_msgs::JointState joint_state;
    joint_state.header.stamp = ros::Time::now();
    joint_state.header.frame_id = "sim";
    for (int i = 0; i < NMOTORS; i++)
    {
        joint_state.name.push_back(armNames[i]);
        joint_state.position.push_back(armpositions[i]);
        joint_state.velocity.push_back(armvelocities[i]);
        joint_state.effort.push_back(0.0);
    }
    ROS_INFO("publishing joint states");
    pub.publish(joint_state);
}

int main(int argc, char **argv) 
{
    std::string controllerName;
    // create a node named 'cssgra' on ROS network
    ros::init(argc, argv, "cssgra_init", ros::init_options::AnonymousName);
    n = new ros::NodeHandle;

    signal(SIGINT, quit);

    // subscribe to the topic model_name to get the list of availables controllers
    ros::Subscriber nameSub = n->subscribe("model_name", 100, controllerNameCallback);
    while (controllerCount == 0 || controllerCount < nameSub.getNumPublishers()) {
        ros::spinOnce();
    }
    ros::spinOnce();

    timeStepClient = n->serviceClient<webots_ros::set_int>("cssgra/robot/time_step");
    timeStepSrv.request.value = TIME_STEP;

    // if there is more than one controller available, it let the user choose
    if (controllerCount == 1)
        controllerName = controllerList[0];
    else {
        int wantedController = 0;
        std::cout << "Choose the # of the controller you want to use:\n";
        std::cin >> wantedController;
        if (1 <= wantedController && wantedController <= controllerCount)
        controllerName = controllerList[wantedController - 1];
        else {
        ROS_ERROR("Invalid number for controller choice.");
        return 1;
        }
    }
    ROS_INFO("Using controller: '%s'", controllerName.c_str());
    // leave topic once it is not necessary anymore
    nameSub.shutdown();

    // 使能所有电机的位置传感器 
    ros::ServiceClient position_sensor_client;
    webots_ros::set_int position_sensor_srv;
    for (int i = 0; i < NMOTORS; i++)
    {
        position_sensor_client = n->serviceClient<webots_ros::set_int>(string("/cssgra/")+string(armNames[i])+string("_sensor/enable"));
        position_sensor_srv.request.value = TIME_STEP;
        if (position_sensor_client.call(position_sensor_srv) && position_sensor_srv.response.success)     
            ROS_INFO("Enabled %s successful.", armNames[i]);   
        else     
            ROS_ERROR("Failed to enabled %s.", armNames[i]);
    }
    ros::Subscriber sub_joint_1_sensor;
    sub_joint_1_sensor = n->subscribe("/cssgra/"+string(armNames[0])+"_sensor/value",1,joint_1_sensorcallback);
    ros::Subscriber sub_joint_2_sensor;
    sub_joint_2_sensor = n->subscribe("/cssgra/"+string(armNames[1])+"_sensor/value",1,joint_2_sensorcallback);
    ros::Subscriber sub_joint_3_sensor;
    sub_joint_3_sensor = n->subscribe("/cssgra/"+string(armNames[2])+"_sensor/value",1,joint_3_sensorcallback);
    ros::Subscriber sub_joint_4_sensor;
    sub_joint_4_sensor = n->subscribe("/cssgra/"+string(armNames[3])+"_sensor/value",1,joint_4_sensorcallback);
    ros::Subscriber sub_joint_5_sensor;
    sub_joint_5_sensor = n->subscribe("/cssgra/"+string(armNames[4])+"_sensor/value",1,joint_5_sensorcallback);
    


    ros::Publisher pub_Joint_State_Publisher;
    pub_Joint_State_Publisher = n->advertise<sensor_msgs::JointState>("/joint_states",5);

    // main loop
    while (ros::ok()) {
        
        if (!timeStepClient.call(timeStepSrv) || !timeStepSrv.response.success) {
        ROS_ERROR("Failed to call service time_step for next step.");
        break;
        }
        Joint_State_Publish(pub_Joint_State_Publisher);
        ros::spinOnce();
    }
    timeStepSrv.request.value = 0;
    timeStepClient.call(timeStepSrv);

    ros::shutdown();
    return 0;
}


void joint_1_sensorcallback(const webots_ros::Float64Stamped::ConstPtr &value){
    ROS_INFO("got No.1");
    armpositions[0] = value->data;
}

void joint_2_sensorcallback(const webots_ros::Float64Stamped::ConstPtr &value){
    ROS_INFO("got No.2");
    armpositions[1] = value->data;
}

void joint_3_sensorcallback(const webots_ros::Float64Stamped::ConstPtr &value){
    ROS_INFO("got No.3");
    armpositions[2] = value->data;
}

void joint_4_sensorcallback(const webots_ros::Float64Stamped::ConstPtr &value){
    ROS_INFO("got No.4");
    armpositions[3] = value->data;
}

void joint_5_sensorcallback(const webots_ros::Float64Stamped::ConstPtr &value){
    ROS_INFO("got No.5");
    armpositions[4] = value->data;
}