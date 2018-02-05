/*
    spidevlib.c - A user-space program to comunicate using spidev.
                Gustavo Zamboni

http://linux-sunxi.org/SPIdev
*/
#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

#include "spi_lib.h"


//////////
// Init SPIdev
//////////
//配置放在script.bin中,这里就不需要配置了。
int spi_init(char filename[40])
{
    int file;
    __u8    mode, lsb, bits;
    __u32 speed=12000000;//12M

    if ((file = open(filename,O_RDWR)) < 0)
    {
        printf("Failed to open the bus.");
        /* ERROR HANDLING; you can check errno to see what went wrong */
       // com_serial=0;
        exit(1);
    }

///////////////
// Verifications
///////////////
//possible modes: mode |= SPI_LOOP; mode |= SPI_CPHA; mode |= SPI_CPOL; mode |= SPI_LSB_FIRST; mode |= SPI_CS_HIGH; mode |= SPI_3WIRE; mode |= SPI_NO_CS; mode |= SPI_READY;
//multiple possibilities using |
/*
    if (ioctl(file, SPI_IOC_WR_MODE, &mode)<0)   {
        perror("can't set spi mode");
        return;
    }
*/
/*
    if (ioctl(file, SPI_IOC_RD_MODE, &mode) < 0)
    {
        perror("SPI rd_mode");
        return;
    }
    if (ioctl(file, SPI_IOC_RD_LSB_FIRST, &lsb) < 0)
    {
        perror("SPI rd_lsb_fist");
        return;
    }
*/
//sunxi supports only 8 bits
/*
    if (ioctl(file, SPI_IOC_WR_BITS_PER_WORD, (__u8[1]){8})<0)
    {
        perror("can't set bits per word");
        return;
    }
*/
/*
    if (ioctl(file, SPI_IOC_RD_BITS_PER_WORD, &bits) < 0)
    {
        perror("SPI bits_per_word");
        return;
    }
*/
/*
    if (ioctl(file, SPI_IOC_WR_MAX_SPEED_HZ, &speed)<0)
    {
        perror("can't set max speed hz");
        return;
    }
*/
/*
    if (ioctl(file, SPI_IOC_RD_MAX_SPEED_HZ, &speed) < 0)
    {
        perror("SPI max_speed_hz");
        return;
    }


   printf("%s: spi mode %d, %d bits %sper word, %d Hz max\n",filename, mode, bits, lsb ? "(lsb first) " : "", speed);
*/
    return file;
}



//////////
// Read n bytes from the 2 bytes add1 add2 address
//////////

void spi_write_read(unsigned char *buf2_w, unsigned char *buf2_r, char nbytes,int file)
{
    int status;
    struct spi_ioc_transfer xfer;

    xfer.tx_buf = (unsigned long)buf2_w;
    xfer.len = nbytes; // Length of  command&Data to write
    xfer.rx_buf=(unsigned long)buf2_r;

    xfer.cs_change=0;
    xfer.speed_hz=12000000;
    xfer.bits_per_word=8;
    xfer.delay_usecs=0;
    status = ioctl(file, SPI_IOC_MESSAGE(1), &xfer);
    if (status < 0)
    {
        perror("SPI_IOC_MESSAGE");
        return;
    }
}
