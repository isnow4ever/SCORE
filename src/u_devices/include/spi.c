/*专门为AD7904写的数据读取函数，并不适用于其他型号的ADC
为了兼容之前用PCdunio做的u_dvice，以下函数的形式写成ardunio的标准形式
*/



#include "marsboard.h"


int8_t analogRead(uint8_t channel)
{
    int file;
    unsigned char buf2_w[2];
    unsigned char buf2_r[2]={0};

    file=spi_init("/dev/spidev0.0"); //dev

/*choose the channel*/
    switch(channel)
    {
        case 0:
            buf2_w[0]=0x83;
            buf2_w[1]=0x00;
            break;
        case 1:
            buf2_w[0]=0x87;
            buf2_w[1]=0x00;
            break;
        case 2:
            buf2_w[0]=0x8f;
            buf2_w[1]=0x80;
            break;
        default :
            break;
    }
    spi_write_read(buf2_w,buf2_r,2,file);//write the command to ad7904

    buf2_w[0]=0x00;
    buf2_w[1]=0x00;//set the value just to read
    spi_write_read(buf2_w,buf2_r,2,file);

 //   printf("data:%x%x\r\n",buf2_r[0],buf2_r[1]);
    if(file)
    close(file);

    if((buf2_r[0]>>4)!=channel)//get the wrong channel
    {
        return 0;
    }

    return ((buf2_r[0]<<4)+(buf2_r[1]>>4)) ;//return value of the specified channel

}


