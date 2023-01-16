#include <ros/ros.h>
#include <std_msgs/String.h>
#include <mars_syncboard/sync.h>

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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "syncboard_button_assist");
    ros::NodeHandle n("~");

    ros::Subscriber sub = n.subscribe("/syncboard/button", 1, button_cb);

    ros::spin();
}