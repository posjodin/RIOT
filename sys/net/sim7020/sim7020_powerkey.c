#include <stdio.h>
#include <stdlib.h>
#include "periph/gpio.h"
#include "xtimer.h"
#include "net/sim7020_powerkey.h"

static gpio_t sim7020_pin = SIM7020_POWERKEY_GPIO;

int sim7020_powerkey_init(void) {
    int res;
    if ((res = gpio_init(sim7020_pin, GPIO_OUT) < 0)) {
        printf("Error initialize sim7020 gpio powerkey: %d\n", res);
    }
    return res;
}

static void _pw_toggle(uint32_t msecs) {
    gpio_set(sim7020_pin);
    xtimer_usleep(msecs*1000);
    gpio_clear(sim7020_pin);
}
  
void sim7020_power_on(void) {
    _pw_toggle(800);
    xtimer_sleep(2);
}

void sim7020_power_off(void) {
    _pw_toggle(2000);
    xtimer_sleep(2);
}
