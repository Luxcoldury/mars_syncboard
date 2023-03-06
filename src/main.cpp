// #include <unistd.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <std_msgs/Time.h>
#include <std_msgs/String.h>
#include <mars_syncboard/sync.h>
#include <mars_syncboard_srvs/ConfigLine.h>
#include <mars_syncboard_srvs/ConfigGPRMC.h>
#include <mars_syncboard_srvs/ToggleTrigger.h>
#include <mars_syncboard_srvs/ToggleButtonLED.h>

bool config_line_from_param();
void update_line_config_param(int line_index);
bool config_gps_from_param();
void update_gps_config_param();
bool config_trigger_from_param();
void update_trigger_param();
bool config_led_from_param();
void update_led_param();
bool config_line(mars_syncboard_srvs::ConfigLine::Request  &req, mars_syncboard_srvs::ConfigLine::Response &res);
bool config_gps(mars_syncboard_srvs::ConfigGPRMC::Request  &req, mars_syncboard_srvs::ConfigGPRMC::Response &res);
bool toggle_trigger(bool trigger);
bool toggle_trigger(mars_syncboard_srvs::ToggleTrigger::Request  &req, mars_syncboard_srvs::ToggleTrigger::Response &res);
bool toggle_button_led(int mode);
bool toggle_button_led(mars_syncboard_srvs::ToggleButtonLED::Request  &req, mars_syncboard_srvs::ToggleButtonLED::Response &res);

void button_callback(int gpio, int level, uint32_t tick);

ros::NodeHandle *nh;
ros::Publisher pub_btn;

int wid_now, wid_next;
int sec_t0, sec_tn, sec_now, micro_now = -1;

bool lines_triggering = false;
int btn_led_mode = BTN_LED_ON;

struct line_config sync_lines[GP_LINE_COUNT]={
    {1,  GPIO_LINE_1 , false, 0, 10, 1, 0, 50},
    {2,  GPIO_LINE_2 , false, 0, 10, 1, 0, 50},
    {8,  GPIO_LINE_8 , false, 0, 10, 1, 0, 50},
    {9,  GPIO_LINE_9 , false, 0, 10, 1, 0, 50},
    {10, GPIO_LINE_10, false, 0, 10, 1, 0, 50},
    {11, GPIO_LINE_11, false, 0, 10, 1, 0, 50},
    {12, GPIO_LINE_12, false, 0, 10, 1, 0, 50},
    {13, GPIO_LINE_13, false, 0, 10, 1, 0, 50},
    {14, GPIO_LINE_14, false, 0, 10, 1, 0, 50},
    {15, GPIO_LINE_15, false, 0, 10, 1, 0, 50},
    {16, GPIO_LINE_16, false, 0, 10, 1, 0, 50},
    {17, GPIO_LINE_17, false, 0, 10, 1, 0, 50}
};

struct gprmc_config gprmc_line={GPIO_LINE_GPS, 9600, 100*1000, false};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "syncboard");

    if(syncboardInit()<0){
        printf("PIGPIO initialization failed.");
        return -1;
    }

    ros::NodeHandle n("~");
    nh=&n;

    ros::ServiceServer service_config_line = n.advertiseService("config_line", config_line);
    ros::ServiceServer service_config_gps = n.advertiseService("config_gps", config_gps);
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

    config_line_from_param();
    config_gps_from_param();
    config_trigger_from_param();
    config_led_from_param();

    // Wait for T-1
    do {gpioTime(1, &sec_now, &micro_now);} while (micro_now > 500000);

    sec_t0 = sec_now + 1;
    wid_now = gpioWavePrepare1sec(sync_lines, GP_LINE_COUNT, gprmc_line, btn_led_mode, sec_t0, sec_t0, false);

    // Wait for T+0
    do {gpioTime(1, &sec_now, &micro_now);} while (sec_now < sec_t0);

    gpioWaveTxSend(wid_now, PI_WAVE_MODE_ONE_SHOT);
    sec_tn = sec_t0;
    if (DEBUG_RT) printf("%d.%06d Sent\n\n", sec_now, micro_now);

    while (ros::ok())
    {

        gpioTime(1, &sec_now, &micro_now);
        while(micro_now<900000){
            ros::spinOnce();
            // ROS_INFO("spin");
            usleep(10000);
            gpioTime(1, &sec_now, &micro_now);
        }

        if (DEBUG_RT) printf("%d.%06d Stop Spin and wait for sending\n\n", sec_now, micro_now);

        wid_next = gpioWavePrepare1sec(sync_lines, GP_LINE_COUNT, gprmc_line, btn_led_mode, sec_t0, sec_tn+1, lines_triggering);

        do {gpioTime(1, &sec_now, &micro_now);} while (sec_now != sec_tn + 1);
        gpioWaveTxSend(wid_next, PI_WAVE_MODE_ONE_SHOT);

        sec_tn = sec_now;
        if (DEBUG_RT) printf("%d.%06d Sent\n\n", sec_now, micro_now);
        ROS_INFO("Running, %s",lines_triggering ? "Triggering" : "IDLING");

        gpioWaveDelete(wid_now);
        wid_now = wid_next;

        std_msgs::Time msg_line_tmp;

        for(int i=0;i<GP_LINE_COUNT;i++){
            if(lines_triggering&&sync_lines[i].enabled){
                for(int j=0;j<sync_lines[i].freq;j++){
                    uint32_t us = 1000000/sync_lines[i].freq*j+sync_lines[i].offset_us;
                    msg_line_tmp.data = ros::Time(sec_tn,1000*us);
                    pub_lines[i].publish(msg_line_tmp);
                }
            }
        }

        config_line_from_param();
        config_gps_from_param();
        config_trigger_from_param();
        config_led_from_param();
    }

    gpioWaveTxStop();

    return 0;
}

void update_line_config_param(int line_index){
    std::string line_root = "line/"+std::to_string(sync_lines[line_index].num);
    nh->setParam(line_root+"/enabled", sync_lines[line_index].enabled);
    nh->setParam(line_root+"/trigger_type", sync_lines[line_index].trigger_type);
    nh->setParam(line_root+"/freq", (double)sync_lines[line_index].freq/sync_lines[line_index].every_n_seconds);
    nh->setParam(line_root+"/offset_us", (int)sync_lines[line_index].offset_us);
    nh->setParam(line_root+"/duty_cycle_percent", sync_lines[line_index].duty_cycle_percent);
}

void update_gps_config_param(){
    nh->setParam("gps/baud", (int)gprmc_line.baud);
    nh->setParam("gps/offset_us", (int)gprmc_line.offset);
    nh->setParam("gps/inverted", gprmc_line.inverted);
}

void update_trigger_param(){
    nh->setParam("triggering", lines_triggering);
}

void update_led_param(){
    nh->setParam("led", btn_led_mode);
}

bool config_line_from_param(){

    for(int i=0;i<GP_LINE_COUNT;i++){
        uint8_t line_num=sync_lines[i].num;
        bool enabled;
        int trigger_type;
        double freq;
        int offset_us;
        int duty_cycle_percent;

        std::string line_root = "line/"+std::to_string(line_num);

        nh->param(line_root+"/enabled", enabled, sync_lines[i].enabled);
        nh->param(line_root+"/trigger_type", trigger_type, sync_lines[i].trigger_type);
        nh->param(line_root+"/freq", freq, (double)sync_lines[i].freq/sync_lines[i].every_n_seconds);
        nh->param(line_root+"/offset_us", offset_us, (int)sync_lines[i].offset_us);
        nh->param(line_root+"/duty_cycle_percent", duty_cycle_percent, sync_lines[i].duty_cycle_percent);

        if(enabled==sync_lines[i].enabled &&
           trigger_type==sync_lines[i].trigger_type &&
           freq==(double)sync_lines[i].freq/sync_lines[i].every_n_seconds &&
           offset_us==(int)sync_lines[i].offset_us &&
           duty_cycle_percent==sync_lines[i].duty_cycle_percent
        ) continue;

        mars_syncboard_srvs::ConfigLine::Request req;
        mars_syncboard_srvs::ConfigLine::Response res;

        req.line_num=line_num;
        req.enabled=enabled;
        req.trigger_type=trigger_type;
        req.freq=freq;
        req.offset_us=offset_us;
        req.duty_cycle_percent=duty_cycle_percent;

        config_line(req,res);
    }

    return true;

}

bool config_gps_from_param(){
    int baud;
    int offset;
    bool inverted;

    nh->param("gps/baud", baud, (int)gprmc_line.baud);
    nh->param("gps/offset_us", offset, (int)gprmc_line.offset);
    nh->param("gps/inverted", inverted, gprmc_line.inverted);

    if(baud==(int)gprmc_line.baud&&
       offset==(int)gprmc_line.offset&&
       inverted==gprmc_line.inverted)
    {
        update_gps_config_param();
        return true;
    }

    mars_syncboard_srvs::ConfigGPRMC::Request  req;
    mars_syncboard_srvs::ConfigGPRMC::Response res;

    req.baud=baud;
    req.offset_us=offset;
    req.inverted=inverted;

    config_gps(req,res);

    return true;
}

bool config_trigger_from_param(){
    bool triggering;
    nh->param("triggering", triggering, lines_triggering);
    if(triggering==lines_triggering){
        update_trigger_param();
        return true;
    }

    toggle_trigger(triggering);
    return true;
}

bool config_led_from_param(){
    int mode;
    nh->param("led", mode, btn_led_mode);
    if(mode==btn_led_mode){
        update_led_param();
        return true;
    }

    toggle_button_led(mode);
    return true;
}

bool config_line(mars_syncboard_srvs::ConfigLine::Request  &req,
                 mars_syncboard_srvs::ConfigLine::Response &res)
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
        res_msg = "Trigger type " + std::to_string(req.trigger_type) + " does not exist";
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

    // Check freq against EITHER_EDGE
    if(req.trigger_type == EITHER_EDGE && freq%2==1){
        res_msg = "Freq="+std::to_string(req.freq)+"Hz does not work with triggering type EITHER_EDGE";
        goto line_config_failed;
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
    update_line_config_param(line_index);
    return true;

    // On failure
    line_config_failed:
        res.succeeded = false;
        res.msg = res_msg;
        ROS_ERROR("Line Config failed. %s", res_msg.c_str());
        if(line_index>=0) update_line_config_param(line_index);
        return true;
}

bool toggle_trigger(bool trigger){
    lines_triggering = trigger;
    if(lines_triggering){
        ROS_INFO("Triggering Started");
    }else{
        ROS_INFO("Triggering Stopped");
    }
    update_trigger_param();
    return true;
}

bool toggle_trigger(mars_syncboard_srvs::ToggleTrigger::Request  &req,
                    mars_syncboard_srvs::ToggleTrigger::Response &res)
{
    toggle_trigger(req.start_trigger);
    res.triggering = lines_triggering;
    return true;
}

bool toggle_button_led(int mode)
{
    btn_led_mode = mode;
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
    update_led_param();
    return true;
}

bool toggle_button_led(mars_syncboard_srvs::ToggleButtonLED::Request  &req,
                       mars_syncboard_srvs::ToggleButtonLED::Response &res)
{
    toggle_button_led(req.mode);
    res.mode = btn_led_mode;
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

bool config_gps(mars_syncboard_srvs::ConfigGPRMC::Request  &req,
                mars_syncboard_srvs::ConfigGPRMC::Response &res)
{
    std::string res_msg;
    int good_baud_rates[7] = {9600,14400,19200,38400,56000,57600,115200};

    for(int i=0;i<7;i++){
        if(req.baud==good_baud_rates[i]) goto baud_rate_checked;
    }
    res_msg = "Baud rate should be (9600,14400,19200,38400,56000,57600,115200)";
    goto gps_config_failed;

    baud_rate_checked:
    if(req.offset_us>900000){
        res_msg = "Offset should be less then 900000us";
        goto gps_config_failed;
    }

    gprmc_line.baud = req.baud;
    gprmc_line.inverted = req.inverted;
    gprmc_line.offset = req.offset_us;

    res_msg = "GPS Config succeeded.";

    // On success
    res.succeeded = true;
    res.msg = res_msg;
    ROS_INFO("%s",res_msg.c_str());
    update_gps_config_param();
    return true;

    // On failure
    gps_config_failed:
        res.succeeded = false;
        res.msg = res_msg;
        ROS_ERROR("GPS Config failed. %s", res_msg.c_str());
        update_gps_config_param();
        return true;
}