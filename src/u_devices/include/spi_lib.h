#ifndef _SPI_LIB_H_
#define _SPI_LIB_H_

int spi_init(char filename[40]);
void spi_write_read(unsigned char *buf2_w, unsigned char *buf2_r, char nbytes,int file);


#endif // _SPI_LIB_H_
