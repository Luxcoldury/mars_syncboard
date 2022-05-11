#include <mars_syncboard/pigpio.h>

#define DEBUG_RT false

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

#define BTN_LED_OFF         0
#define BTN_LED_ON          1
#define BTN_LED_BLINK_1HZ   2

struct line_config{
    int num;
    int gpio;
    bool enabled;
    int trigger_type;
    unsigned freq;
    unsigned every_n_seconds;
    unsigned offset_us;
    int duty_cycle_percent;
};

struct gprmc_config{
    int gpio;
    unsigned baud;
    unsigned offset;
    bool inverted; 
};

int syncboardInit();
int gpioWavePrepare1sec(struct line_config sync_lines[], int line_count, struct gprmc_config, int btn_led_mode, int sec_t0, int sec_to_prepare, bool lines_triggering);
int gpioWaveAddGprmc(unsigned gpio, unsigned baud, unsigned offset, time_t timestamp, int inverted);
int gpioWaveAddGprmc(struct gprmc_config, time_t timestamp);
int gpioWaveAddFreq1sec(int gpio, int trigger_type, unsigned freq, unsigned offset_us, int duty_cycle_percent);
int gpioWaveAddFreq1sec(struct line_config);