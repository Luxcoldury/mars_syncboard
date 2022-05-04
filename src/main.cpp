#include "ros/ros.h"
#include "std_msgs/String.h"
#include <mars_syncboard/sync.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test");

    int wid_now, wid_next;
    int sec_first, sec_now, sec_new, micro_now = -1;
    
    if(syncboardInit()<0) return -1;

    ros::NodeHandle n;

    ros::Rate loop_rate(1);

    ros::Publisher pub = n.advertise<std_msgs::String>("time", 5);

    while (!ros::ok())
    {
        sleep(1);
    }

    // Wait for T-1
    do {gpioTime(1, &sec_now, &micro_now);} while (micro_now > 500000);

    sec_first = sec_now + 1;
    wid_now = gpioWavePrepare1sec(sec_first, sec_first);

    // Wait for T+0
    do {gpioTime(1, &sec_new, &micro_now);} while (sec_new < sec_first);

    gpioWaveTxSend(wid_now, PI_WAVE_MODE_ONE_SHOT);
    sec_now = sec_new;
    if (DEBUG_RT) printf("%d.%06d Sent\n\n", sec_now, micro_now);

    while (ros::ok())
    {

        std_msgs::String str;
        str.data = "hello world";
        ROS_INFO("%s", str.data.c_str());
        pub.publish(str);

        ros::spinOnce();

        wid_next = gpioWavePrepare1sec(sec_first, sec_now + 1);

        do {gpioTime(1, &sec_new, &micro_now);} while (sec_new != sec_now + 1);
        gpioWaveTxSend(wid_next, PI_WAVE_MODE_ONE_SHOT);

        sec_now = sec_new;
        if (DEBUG_RT) printf("%d.%06d Sent\n\n", sec_now, micro_now);

        gpioWaveDelete(wid_now);
        wid_now = wid_next;
    }

    gpioWaveTxStop();

    return 0;
}