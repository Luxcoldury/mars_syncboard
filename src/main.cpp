// #include <unistd.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <std_msgs/Time.h>
#include <std_msgs/String.h>
#include <mars_syncboard/sync.h>
#include <mars_syncboard/ConfigLine.h>
#include <mars_syncboard/ToggleTrigger.h>
#include <mars_syncboard/ToggleButtonLED.h>

bool config_line(mars_syncboard::ConfigLine::Request  &req, mars_syncboard::ConfigLine::Response &res);
bool toggle_trigger(mars_syncboard::ToggleTrigger::Request  &req, mars_syncboard::ToggleTrigger::Response &res);
bool toggle_button_led(mars_syncboard::ToggleButtonLED::Request  &req, mars_syncboard::ToggleButtonLED::Response &res);

void button_callback(int gpio, int level, uint32_t tick);

ros::Publisher pub_btn;

int wid_now, wid_next;
int sec_now, sec_t0, sec_tn, micro_now, micro_last = -1;
bool lines_triggering = false;
int btn_led_mode = BTN_LED_ON;

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

    if(syncboardInit()<0){
        printf("PIGPIO initialization failed.");
        return -1;
    }

    ros::NodeHandle n("~");

    ros::ServiceServer service_config_line = n.advertiseService("config_line", config_line);
    ros::ServiceServer service_toggle_trigger = n.advertiseService("toggle_trigger", toggle_trigger);
    ros::ServiceServer service_toggle_button_led = n.advertiseService("toggle_button_led", toggle_button_led);
    
    std::vector<ros::Publisher> pub_lines;
    for(int i=0;i<GP_LINE_COUNT;i++){    
        ros::Publisher pub_line_tmp = n.advertise<std_msgs::Time>("line/"+std::to_string(sync_lines[i].num), 1100);
        pub_lines.push_back(pub_line_tmp);
    }

    pub_btn = n.advertise<std_msgs::String>("button", 1000);
    gpioGlitchFilter(GPIO_LINE_BTN, 100000);
    gpioSetAlertFunc(GPIO_LINE_BTN, button_callback);

    while (!ros::ok())
    {
        sleep(1);
    }

    // Wait for T-1
    do {gpioTime(1, &sec_now, &micro_now);} while (micro_now > 500000);

    sec_t0 = sec_now + 1;
    sec_tn = sec_t0;
    wid_now = gpioWavePrepare1sec(sync_lines, GP_LINE_COUNT, sec_t0, sec_tn, false, btn_led_mode);

    // Wait for T+0
    do {gpioTime(1, &sec_now, &micro_now);} while (sec_now < sec_t0);

    gpioWaveTxSend(wid_now, PI_WAVE_MODE_ONE_SHOT_SYNC);
    if (DEBUG_RT) printf("%d Sent\n\n", sec_tn);

    micro_last = micro_now;
    sec_tn ++;

    while (ros::ok())
    {

        gpioTime(1, &sec_now, &micro_now);
        while((micro_now-micro_last+1000000)%1000000<900000){
            ros::spinOnce();
            // ROS_INFO("spin");
            usleep(10000);
            gpioTime(1, &sec_now, &micro_now);
        }

        if (DEBUG_RT) printf("%06d Stop Spin and wait for sending\n\n", micro_now-micro_last);

        wid_next = gpioWavePrepare1sec(sync_lines, GP_LINE_COUNT, sec_t0, sec_tn, lines_triggering, btn_led_mode);

        do {gpioTime(1, &sec_now, &micro_now);} while (gpioWaveTxAt()==wid_now);
        gpioWaveTxSend(wid_next, PI_WAVE_MODE_ONE_SHOT_SYNC);

        if (DEBUG_RT) printf("%d Sent\n\n", sec_tn);

        micro_last = micro_now;
        sec_tn ++;

        ROS_INFO("Running, %s",lines_triggering ? "Triggering" : "Not Triggering");

        gpioWaveDelete(wid_now);
        wid_now = wid_next;

        std_msgs::Time msg_line_tmp;

        for(int i=0;i<GP_LINE_COUNT;i++){
            if(lines_triggering&&sync_lines[i].enabled){
                for(int j=0;j<sync_lines[i].freq;j++){
                    uint32_t us = 1000000/sync_lines[i].freq*j+sync_lines[i].offset_us;
                    msg_line_tmp.data = ros::Time(sec_now,1000*us);
                    pub_lines[i].publish(msg_line_tmp);
                }
            }
        }
        
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

bool toggle_button_led(mars_syncboard::ToggleButtonLED::Request  &req,
                       mars_syncboard::ToggleButtonLED::Response &res)
{
    btn_led_mode = req.mode;
    res.mode = btn_led_mode;
    switch(btn_led_mode){
        case BTN_LED_OFF:
            ROS_INFO("Button LED: OFF");
            break;
        case BTN_LED_ON:
            ROS_INFO("Button LED: ON");
            break;
        case BTN_LED_BLINK_1HZ:
            ROS_INFO("Button LED: BLINK at 1Hz");
            break;
    }
    return true;
}

void button_callback(int gpio, int level, uint32_t tick){
    std_msgs::String msg;
    if(gpio == GPIO_LINE_BTN){
        switch(level){
            case 0: // falling edge
                msg.data = "Released";
                break;
            case 1: // rising edge
                msg.data = "Pressed";
                break;
        }
        ROS_INFO("Button %s", msg.data.c_str());
        pub_btn.publish(msg);
    }
}