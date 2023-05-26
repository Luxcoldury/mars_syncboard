#include <ros/ros.h>
#include <std_msgs/String.h>
#include <mars_syncboard/sync.h>
#include <mars_syncboard_srvs/BoardStatus.h>

void button_cb(const std_msgs::String::ConstPtr& msg)
{
    if(msg->data=="Released"){
        bool triggering;
        ros::param::get("/syncboard/triggering",triggering);
        if(triggering){
            ros::param::set("/syncboard/triggering",false);
            ros::param::set("/syncboard/led",BTN_LED_ON);
        }else{
            ros::param::set("/syncboard/triggering",true);
            ros::param::set("/syncboard/led",BTN_LED_BLINK_1HZ);
        }
    }
}

void status_cb(const mars_syncboard_srvs::BoardStatus::ConstPtr& msg)
{
    if(msg->triggering){
        ros::param::set("/syncboard/led",BTN_LED_BLINK_1HZ);
    }else{
        ros::param::set("/syncboard/led",BTN_LED_ON);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "syncboard_button_assist");
    ros::NodeHandle n("~");

    ros::Subscriber sub_btn = n.subscribe("/syncboard/button", 1, button_cb);
    ros::Subscriber sub_status = n.subscribe("/syncboard/status", 1, status_cb);

    ros::spin();
}