#include <mars_syncboard/pigpio.h>

#define DEBUG_RT true

#define GPIO_LINE_1         2
#define GPIO_LINE_2         3
#define GPIO_LINE_BTN       4
#define GPIO_LINE_BTN_LED   5
#define GPIO_LINE_PPS       6
#define GPIO_LINE_8         8
#define GPIO_LINE_9         9
#define GPIO_LINE_10        10
#define GPIO_LINE_11        11
#define GPIO_LINE_12        12
#define GPIO_LINE_13        13
#define GPIO_LINE_GPS       14
#define GPIO_LINE_14        16
#define GPIO_LINE_15        17
#define GPIO_LINE_16        18
#define GPIO_LINE_17        19

#define GP_LINE_COUNT       12

struct line_config{
    int num;
    int gpio;
    int enabled;
    int trigger_type;
    unsigned freq;
    unsigned every_n_seconds;
    unsigned offset_us;
    int duty_cycle_percent;
};

int syncboardInit();
int gpioWavePrepare1sec(int sec_first, int sec_to_prepare);
int gpioWavePrepare1sec(struct line_config sync_lines[], int line_count, int sec_first, int sec_to_prepare);
int gpioWaveAddGprmc(unsigned gpio, unsigned offset, time_t timestamp, int inverted);
int gpioWaveAddFreq1sec(int gpio, int trigger_type, unsigned freq, unsigned offset_us, int duty_cycle_percent);
int gpioWaveAddFreq1sec(struct line_config);