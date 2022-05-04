#include "ros/ros.h"
#include "std_msgs/String.h"
#include "pigpio.h"

#define DEBUG_RT true

int gpio_line_1 = 2;
int gpio_pps = 6;
int gpio_gps = 14;

int gpioWavePrepare1sec(int sec_first, int sec_to_prepare);
int gpioWaveAddGprmc(unsigned gpio, unsigned offset, time_t timestamp, int inverted);
int gpioWaveAddFreq1sec(int gpio, int trigger_type, unsigned freq, unsigned offset_us, int duty_cycle_percent);

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "test");

  if (gpioInitialise()<0) return -1;
  gpioSetMode(gpio_line_1, PI_OUTPUT);
  gpioSetMode(gpio_pps, PI_OUTPUT);
  gpioSetMode(gpio_gps, PI_OUTPUT);

  int wid_now, wid_next;
  int sec_first, sec_now, sec_new, micro_now = -1;
    

  ros::NodeHandle n;

  ros::Rate loop_rate(1);

  ros::Publisher pub = n.advertise<std_msgs::String>("time", 5);


  while(!ros::ok()){
    sleep(1);
  }

  // Wait for T-1
  do{gpioTime(1,&sec_now,&micro_now);}while(micro_now>500000);

  sec_first = sec_now+1;
  wid_now = gpioWavePrepare1sec(sec_first,sec_first);

  // Wait for T+0 
  do{gpioTime(1,&sec_new,&micro_now);}while(sec_new<sec_first);

  gpioWaveTxSend(wid_now, PI_WAVE_MODE_ONE_SHOT);
  sec_now = sec_new;
  if(DEBUG_RT) printf("%d.%06d Sent\n\n",sec_now,micro_now);
  
  while (ros::ok())
  {

    std_msgs::String str;
    str.data = "hello world";
    ROS_INFO("%s", str.data.c_str());
    pub.publish(str);

    ros::spinOnce();

    wid_next = gpioWavePrepare1sec(sec_first,sec_now+1);

    do{gpioTime(1,&sec_new,&micro_now);}while(sec_new!=sec_now+1);
    gpioWaveTxSend(wid_next, PI_WAVE_MODE_ONE_SHOT);

    sec_now = sec_new;
    if(DEBUG_RT) printf("%d.%06d Sent\n\n",sec_now,micro_now);

    gpioWaveDelete(wid_now);
    wid_now = wid_next;

  }

  gpioWaveTxStop();

  return 0;
}




int gpioWavePrepare1sec(int sec_first, int sec_to_prepare){
    if(DEBUG_RT) printf("Preparing for %d\n",sec_to_prepare);
    // if((sec_to_prepare-sec_first)%20==0) gpioWaveAddFreq1sec(gpio_pps,RISING_EDGE,1,0,10);
    gpioWaveAddFreq1sec(gpio_line_1,RISING_EDGE,10,0,50);

    // Lidar Timestamp
    gpioWaveAddFreq1sec(gpio_pps,RISING_EDGE,1,0,5);
    gpioWaveAddGprmc(gpio_gps,100*1000,sec_to_prepare,false);


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