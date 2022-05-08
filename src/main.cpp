#include "ros/ros.h"
#include "std_msgs/String.h"
#include <mars_syncboard/sync.h>

int wid_now, wid_next;
int sec_first, sec_now, sec_new, micro_now = -1;
struct line_config sync_lines[GP_LINE_COUNT]={
    {1,  GPIO_LINE_1,  false},
    {2,  GPIO_LINE_2,  false},
    {8,  GPIO_LINE_8,  false},
    {9,  GPIO_LINE_9,  false},
    {10, GPIO_LINE_10, false},
    {11, GPIO_LINE_11, false},
    {12, GPIO_LINE_12, false},
    {13, GPIO_LINE_13, false},
    {14, GPIO_LINE_14, false},
    {15, GPIO_LINE_15, false},
    {16, GPIO_LINE_16, false},
    {17, GPIO_LINE_17, false}
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test");

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
    // wid_now = gpioWavePrepare1sec(sec_first, sec_first);
    wid_now = gpioWavePrepare1sec(sync_lines, GP_LINE_COUNT, sec_first, sec_first);

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

        // wid_next = gpioWavePrepare1sec(sec_first, sec_now + 1);
        wid_next = gpioWavePrepare1sec(sync_lines, GP_LINE_COUNT, sec_first, sec_now + 1);

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