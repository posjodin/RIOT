#ifndef SIM7020_POWERKEY_H
#define SIM7020_POWERKEY_H

#ifndef SIM7020_POWERKEY_GPIO
#define SIM7020_POWERKEY_GPIO GPIO_PIN(1, 2)
#endif 

int sim7020_powerkey_init(void);
void sim7020_power_on(void);
void sim7020_power_off(void);
#endif /* SIM7020_POWERKEY_H */
