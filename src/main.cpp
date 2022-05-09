#include <ros/ros.h>
#include <std_msgs/String.h>
#include <mars_syncboard/sync.h>
#include <mars_syncboard/ConfigLine.h>
#include <mars_syncboard/ToggleTrigger.h>

bool config_line(mars_syncboard::ConfigLine::Request  &req, mars_syncboard::ConfigLine::Response &res);
bool toggle_trigger(mars_syncboard::ToggleTrigger::Request  &req, mars_syncboard::ToggleTrigger::Response &res);

int wid_now, wid_next;
int sec_first, sec_now, sec_new, micro_now = -1;
bool lines_triggering = false;

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
    ros::init(argc, argv, "syncboard");

    if(syncboardInit()<0) return -1;

    ros::NodeHandle n("~");

    ros::ServiceServer service_config_line = n.advertiseService("config_line", config_line);
    ros::ServiceServer service_toggle_trigger = n.advertiseService("toggle_trigger", toggle_trigger);
    // ros::Publisher pub = n.advertise<std_msgs::String>("time", 5);

    while (!ros::ok())
    {
        sleep(1);
    }

    // Wait for T-1
    do {gpioTime(1, &sec_now, &micro_now);} while (micro_now > 500000);

    sec_first = sec_now + 1;
    // wid_now = gpioWavePrepare1sec(sec_first, sec_first);
    wid_now = gpioWavePrepare1sec(sync_lines, GP_LINE_COUNT, sec_first, sec_first, lines_triggering);

    // Wait for T+0
    do {gpioTime(1, &sec_new, &micro_now);} while (sec_new < sec_first);

    gpioWaveTxSend(wid_now, PI_WAVE_MODE_ONE_SHOT);
    sec_now = sec_new;
    if (DEBUG_RT) printf("%d.%06d Sent\n\n", sec_now, micro_now);

    while (ros::ok())
    {

        // std_msgs::String str;
        // str.data = "hello world";
        // ROS_INFO("%s", str.data.c_str());
        // pub.publish(str);

        ros::spinOnce();

        // wid_next = gpioWavePrepare1sec(sec_first, sec_now + 1);
        wid_next = gpioWavePrepare1sec(sync_lines, GP_LINE_COUNT, sec_first, sec_now + 1, lines_triggering);

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

bool config_line(mars_syncboard::ConfigLine::Request  &req,
                 mars_syncboard::ConfigLine::Response &res)
{
    std::string res_msg;

    // Get and check Line num
    int line_index = -1;
    for(int i=0;i<GP_LINE_COUNT;i++){
        if(sync_lines[i].num == req.line_num){
            line_index = i;
            break;
        }
    }
    if(line_index<0){
        res_msg = "Line " + std::to_string(req.line_num) + " does not exist";
        goto line_config_failed;
    }

    // Check Triggering mode
    if(!(req.trigger_type == FALLING_EDGE || req.trigger_type == RISING_EDGE || req.trigger_type == EITHER_EDGE)){
        res_msg = "Trigger type" + std::to_string(req.trigger_type) + "does not exist";
        goto line_config_failed;
    }

    // Calculate and check Freq
    if(req.freq>1000||req.freq<=0){
        res_msg = "Freq should be (0,1000] Hz";
        goto line_config_failed;
    }
    unsigned freq, every_n_seconds;
    if(req.freq>1){
        every_n_seconds = 1;
        freq = (int)req.freq;
    }else{
        every_n_seconds = (int)(1/req.freq);
        freq = 1;
    }

    // Check duty cycle
    if(req.duty_cycle_percent>=100 || req.duty_cycle_percent <=0){
        res_msg = "Duty cycle should be 1-99 %";
        goto line_config_failed;
    }

    // Check offset
    if(every_n_seconds==1){
        int offset_threshold = 1000000/freq;
        if(offset_threshold < req.offset_us){
            res_msg = "Offset should be less than " + std::to_string(offset_threshold) + "us given the freq of " + std::to_string(freq)+ "Hz";
            goto line_config_failed;
        }
    }else{
        int offset_threshold = 1000000*(100 - req.duty_cycle_percent)/100;
        if(offset_threshold < req.offset_us){
            res_msg = "Offset should be less than " + std::to_string(offset_threshold) + "us given the freq of " + std::to_string(freq)+ "Hz and the signal duration of " + std::to_string(1000000-offset_threshold) + "us";
            goto line_config_failed;
        }
    }

    // Set configs
    sync_lines[line_index].enabled              = req.enabled;
    sync_lines[line_index].trigger_type         = req.trigger_type;
    sync_lines[line_index].freq                 = freq;
    sync_lines[line_index].every_n_seconds      = every_n_seconds;
    sync_lines[line_index].offset_us            = req.offset_us;
    sync_lines[line_index].duty_cycle_percent   = req.duty_cycle_percent;

    // Generate success msg
    res_msg = "Line Config succeeded. Line "+std::to_string(req.line_num)+": ";
    res_msg += req.enabled ? "Enabled, " : "Disabled, ";
    if(every_n_seconds == 1){
        res_msg += "freq="+std::to_string(freq)+"Hz, ";
    }else{
        res_msg += "trigger once every "+std::to_string(every_n_seconds)+" seconds, ";
    }
    res_msg += "trigger at ";
    switch(req.trigger_type){
        case RISING_EDGE:
            res_msg += "RISING_EDGE";
            break;
        case FALLING_EDGE:
            res_msg += "FALLING_EDGE";
            break;
        case EITHER_EDGE:
            res_msg += "EITHER_EDGE";
            break;
    }
    res_msg += " with "+std::to_string(req.offset_us)+"us offset, ";
    res_msg += "duty cycle = "+std::to_string(req.duty_cycle_percent)+"%";

    // On success
    res.succeeded = true;
    res.msg = res_msg;
    ROS_INFO("%s",res_msg.c_str());
    return true;

    // On failure
    line_config_failed:
        res.succeeded = false;
        res.msg = res_msg;
        ROS_ERROR("Line Config failed. %s", res_msg.c_str());
        return true;
}

bool toggle_trigger(mars_syncboard::ToggleTrigger::Request  &req,
                    mars_syncboard::ToggleTrigger::Response &res)
{
    lines_triggering = req.start_trigger;
    res.triggering = lines_triggering;
    if(lines_triggering){
        ROS_INFO("Triggering Started");
    }else{
        ROS_INFO("Triggering Stopped");
    }
    return true;
}