#include "marsboard.h"

//GPIO输出，置0或置1
void digitalWrite(uint16_t pin, uint8_t value)
{
    if(SETUP_OK!=sunxi_gpio_init())
    {
        printf("Failed to initialize GPIO\n");
      //  return -1;
    }
    if(SETUP_OK!=sunxi_gpio_set_cfgpin(pin,OUTPUT))
    {
        printf("Failed to config GPIO pin\n");
   //     return -1;
    }
    if(sunxi_gpio_output(pin,value))
    {
        printf("Failed to set GPIO pin value\n");
       // return -1;
    }
    sunxi_gpio_cleanup();
}
//读取GPIO的值
int digitalRead(uint16_t pin)
{
    uint8_t value=0;
    if(SETUP_OK!=sunxi_gpio_init())
    {
        printf("Failed to initialize GPIO\n");
        return -1;
    }
    if(SETUP_OK!=sunxi_gpio_set_cfgpin(pin,INPUT))
    {
        printf("Failed to config GPIO pin\n");
        return -1;
    }
    value=sunxi_gpio_input(pin);
    if(value==-1)
    {
        printf("Failed to set GPIO pin value\n");
        return -1;
    }
    sunxi_gpio_cleanup();
    return value;
}





