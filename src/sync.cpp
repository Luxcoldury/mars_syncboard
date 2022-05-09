#include "ros/ros.h"
#include "std_msgs/String.h"
#include <mars_syncboard/sync.h>

int syncboardInit(){
    if (gpioInitialise()<0) return -1;

    gpioSetMode(GPIO_LINE_1,        PI_OUTPUT);
    gpioSetMode(GPIO_LINE_2,        PI_OUTPUT);
    gpioSetMode(GPIO_LINE_BTN_LED,  PI_OUTPUT);
    gpioSetMode(GPIO_LINE_PPS,      PI_OUTPUT);
    gpioSetMode(GPIO_LINE_8,        PI_OUTPUT);
    gpioSetMode(GPIO_LINE_9,        PI_OUTPUT);
    gpioSetMode(GPIO_LINE_10,       PI_OUTPUT);
    gpioSetMode(GPIO_LINE_11,       PI_OUTPUT);
    gpioSetMode(GPIO_LINE_12,       PI_OUTPUT);
    gpioSetMode(GPIO_LINE_13,       PI_OUTPUT);
    gpioSetMode(GPIO_LINE_GPS,      PI_OUTPUT);
    gpioSetMode(GPIO_LINE_14,       PI_OUTPUT);
    gpioSetMode(GPIO_LINE_15,       PI_OUTPUT);
    gpioSetMode(GPIO_LINE_16,       PI_OUTPUT);
    gpioSetMode(GPIO_LINE_17,       PI_OUTPUT);

    gpioSetMode(GPIO_LINE_BTN,      PI_INPUT);
    gpioSetPullUpDown(GPIO_LINE_BTN, PI_PUD_DOWN);
    return 0;
}

int gpioWavePrepare1sec(int sec_first, int sec_to_prepare){
    if(DEBUG_RT) printf("Preparing for %d\n",sec_to_prepare);
    // if((sec_to_prepare-sec_first)%20==0) gpioWaveAddFreq1sec(gpio_pps,RISING_EDGE,1,0,10);
    gpioWaveAddFreq1sec(GPIO_LINE_1,RISING_EDGE,10,0,50);

    // Lidar Timestamp
    gpioWaveAddFreq1sec(GPIO_LINE_PPS,RISING_EDGE,1,0,5);
    gpioWaveAddGprmc(GPIO_LINE_GPS,100*1000,sec_to_prepare,true);

    // if(gpioWaveGetMicros()!=1000000) printf("[ERROR]Signal length not exactly 1 sec\n");

    return gpioWaveCreatePad(50, 50, 0);
}

int gpioWavePrepare1sec(struct line_config sync_lines[], int line_count, int sec_first, int sec_to_prepare, bool lines_triggering, int btn_led_mode){
    if(DEBUG_RT) printf("Preparing for %d\n",sec_to_prepare);
    // if((sec_to_prepare-sec_first)%20==0) gpioWaveAddFreq1sec(gpio_pps,RISING_EDGE,1,0,10);

    if(lines_triggering){
        for(int i=0;i<line_count;i++){
            if(sync_lines[i].enabled && (sec_to_prepare - sec_first) % sync_lines[i].every_n_seconds == 0){
                gpioWaveAddFreq1sec(sync_lines[i]);
            }
        }
    }

    // Button LED
    switch(btn_led_mode){
        case BTN_LED_OFF:
            gpioWrite(GPIO_LINE_BTN_LED,0);
            break;
        case BTN_LED_ON:
            gpioWrite(GPIO_LINE_BTN_LED,1);
            break;
        case BTN_LED_BLINK_1HZ:
            gpioWaveAddFreq1sec(GPIO_LINE_BTN_LED,RISING_EDGE,1,0,5);
            break;
    }

    // Lidar Timestamp
    gpioWaveAddFreq1sec(GPIO_LINE_PPS,RISING_EDGE,1,0,5);
    gpioWaveAddGprmc(GPIO_LINE_GPS,100*1000,sec_to_prepare,true);

    // if(gpioWaveGetMicros()!=1000000) printf("[ERROR]Signal length not exactly 1 sec\n");

    return gpioWaveCreatePad(50, 50, 0);
}

int gpioWaveAddGprmc(unsigned gpio, unsigned offset, time_t timestamp, int inverted){

    char gprmc_raw[59];
    char gprmc[70];
    int gprmc_checksum = 0;

    struct tm *timestamp_tm;
    timestamp_tm = localtime(&timestamp);

    strftime(gprmc_raw,59,"GPRMC,%H%M%S.00,A,0000.0000,N,0000.0000,E,0.00,,%d%m%y,,,A",timestamp_tm);

    for(int i=0; i<59; i++)
    {
        gprmc_checksum ^= (int)gprmc_raw[i];
    }

    sprintf(gprmc,"$%s*%X",gprmc_raw,gprmc_checksum);
    ROS_DEBUG("GPS serial: %s",gprmc);

    if(inverted){
        gpioWaveAddSerialInverted(gpio,9600,8,2,offset,strlen(gprmc),gprmc);
    }else{
        gpioWaveAddSerial(gpio,9600,8,2,offset,strlen(gprmc),gprmc);
    }

    return 0;
}

int gpioWaveAddFreq1sec(int gpio, int trigger_type, unsigned freq, unsigned offset_us, int duty_cycle_percent){
    gpioPulse_t pulses[2100];
    unsigned cycle_length;
    unsigned pulse_count;

    if(freq>1000){
        ROS_ERROR("GPIO %d : Freq>1000Hz, signal not generated",gpio);
        return -1;
    }        
    
    if(1000000/freq < offset_us){
        ROS_ERROR("GPIO %d : Offset > 1 cycle, signal not generated",gpio);
        return -1;
    }

    if(trigger_type == EITHER_EDGE){
        if(freq%2 == 1) {
            ROS_ERROR("GPIO %d : Odd Freq on EITHER_EDGE triggering, signal not generated",gpio);
            return -1;
        }
        cycle_length = 2000000 / freq;
    }else{
        cycle_length = 1000000 / freq;
    }

    if(offset_us>0){
        if(trigger_type == FALLING_EDGE){
            pulses[0].gpioOn  = (1<<gpio);
            pulses[0].gpioOff = 0;
            pulses[0].usDelay = offset_us;
        }else{
            pulses[0].gpioOn  = 0;
            pulses[0].gpioOff = (1<<gpio);
            pulses[0].usDelay = offset_us;
        }
    }

    switch(trigger_type){
        case RISING_EDGE:
            for(int i=0;i<freq;i++){
                pulses[2*i+1].gpioOn  = (1<<gpio);
                pulses[2*i+1].gpioOff = 0;
                pulses[2*i+1].usDelay = cycle_length*duty_cycle_percent/100;

                pulses[2*i+2].gpioOn  = 0;
                pulses[2*i+2].gpioOff = (1<<gpio);
                pulses[2*i+2].usDelay = cycle_length-cycle_length*duty_cycle_percent/100;
            }

            pulses[2*freq].usDelay = 1000000 - (freq-1)*cycle_length - cycle_length*duty_cycle_percent/100 - offset_us;
            pulse_count = freq*2+1;
        break;

        case FALLING_EDGE:
            for(int i=0;i<freq;i++){
                pulses[2*i+1].gpioOn  = 0;
                pulses[2*i+1].gpioOff = (1<<gpio);
                pulses[2*i+1].usDelay = cycle_length*duty_cycle_percent/100;

                pulses[2*i+2].gpioOn  = (1<<gpio);
                pulses[2*i+2].gpioOff = 0;
                pulses[2*i+2].usDelay = cycle_length-cycle_length*duty_cycle_percent/100;
            }

            pulses[2*freq].usDelay = 1000000 - (freq-1)*cycle_length - cycle_length*duty_cycle_percent/100 - offset_us;
            pulse_count = freq*2+1;
        break;

        case EITHER_EDGE:
            for(int i=0;i<freq/2;i++){
                pulses[2*i+1].gpioOn  = (1<<gpio);
                pulses[2*i+1].gpioOff = 0;
                pulses[2*i+1].usDelay = cycle_length/2;

                pulses[2*i+2].gpioOn  = 0;
                pulses[2*i+2].gpioOff = (1<<gpio);
                pulses[2*i+2].usDelay = cycle_length-cycle_length/2;
            }

            pulses[freq].usDelay = 1000000 - (freq/2-1)*cycle_length - cycle_length/2 - offset_us;
            pulse_count = freq+1;
        break;
    }
    
    gpioWaveAddGeneric(pulse_count, pulses);
    return 0;
}

int gpioWaveAddFreq1sec(struct line_config lc){
    return gpioWaveAddFreq1sec(lc.gpio, lc.trigger_type, lc.freq, lc.offset_us, lc.duty_cycle_percent);
}