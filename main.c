/*
	Comple：make
	Run: ./bme280

	This Demo is tested on Raspberry PI 3B+
	you can use I2C or SPI interface to test this Demo
	When you use I2C interface,the default Address in this demo is 0X77
	When you use SPI interface,PIN 27 define SPI_CS
*/
#include "bme280.h"
#include <stdio.h>
#include <unistd.h>
#include <getopt.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/ioctl.h>
//wpi- #include <wiringPi.h>
//wpi- #include <wiringPiSPI.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

//Raspberry 3B+ platform's default SPI channel
#define channel 0

// KRM board spi channel
char *spidev = "/dev/spidev0.0"; // chipselect 0 on bus 1
int spifd = -1;  // file descriptor for SPI device
#define SPEED_HZ 2000000
//#define SPEED_HZ 1000000

//Default write it to the register in one time
#define USESPISINGLEREADWRITE 0

//This definition you use I2C or SPI to drive the bme280
//When it is 1 means use I2C interface, When it is 0,use SPI interface
#define USEIIC 0

int debug = 0;

#if(USEIIC)
#include <string.h>
#include <stdlib.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <fcntl.h>
//Raspberry 3B+ platform's default I2C device file
#define IIC_Dev  "/dev/i2c-1"

int fd;

void user_delay_ms(uint32_t period)
{
	usleep(period*1000);
}

int8_t user_i2c_read(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
	write(fd, &reg_addr,1);
	read(fd, data, len);
	return 0;
}

int8_t user_i2c_write(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
	int8_t *buf;
	buf = malloc(len +1);
	buf[0] = reg_addr;
	memcpy(buf +1, data, len);
	write(fd, buf, len +1);
	free(buf);
	return 0;
}
#else

/* wpi-  these are handled automatically by read and write operations
void SPI_BME280_CS_High(void)
{
	digitalWrite(27,1);
}

void SPI_BME280_CS_Low(void)
{
	digitalWrite(27,0);
}
*/

void user_delay_ms(uint32_t period)
{
	usleep(period*1000);
}

void linux_spi_read(uint8_t *data, uint16_t len)
{
	ssize_t recv_bytes = read(spifd, data, len);
	if (recv_bytes != len) {
		fprintf(stderr, "Reading from %s, only received %ld bytes not %d\n",
		        spidev, recv_bytes, len);
	}
}

void linux_spi_write(uint8_t *data, uint16_t len)
{
	size_t wrote_bytes = write(spifd, data, len);
	if (wrote_bytes != len) {
		fprintf(stderr, "Writing to %s, only received %ld bytes not %d\n",
		        spidev, wrote_bytes, len);
	}
}

#define SPI_RDBUF_LEN 32 // 1 addr byte, up to 15 data bytes
// See https://github.com/torvalds/linux/blob/master/include/uapi/linux/spi/spidev.h
// or /usr/include/linux/spi/spidev.h
int8_t user_spi_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
	int8_t rslt = 0;

	//wpi- The Linux spi driver handles the chip-select line for us.
	//wpi- SPI_BME280_CS_High();
	//wpi- SPI_BME280_CS_Low();
	//wpi- wiringPiSPIDataRW(channel,&reg_addr,1);

	if (len + 1 > SPI_RDBUF_LEN) {
		fprintf(stderr, "OOPS, cannot handle read > %d bytes. %d requested.\n",
			SPI_RDBUF_LEN-1, len);
		return -1;
	}

	struct spi_ioc_transfer	xfer;
	memset(&xfer, 0, sizeof xfer);
	uint8_t wbuf[SPI_RDBUF_LEN];
	memset(wbuf, 0, sizeof wbuf);
	uint8_t rbuf[SPI_RDBUF_LEN];
	memset(rbuf, 0, sizeof rbuf);
	xfer.len = len + 1;
	xfer.rx_buf = (__u64) rbuf;
	xfer.tx_buf = (__u64) wbuf;

	wbuf[0] = reg_addr | 0x80; // high bit = 1 for read (0 for write)
	int status = (int8_t) ioctl(spifd, SPI_IOC_MESSAGE(1), &xfer);
	if (status == -1) {
		fprintf(stderr, "ERROR in ioctl: %s\n", strerror(errno));
		return -1;
	}
	memcpy(reg_data, rbuf + 1, len);

	//wpi- #if(USESPISINGLEREADWRITE)
	//wpi- for(int i=0; i < len ; i++)
	//wpi- {
	//wpi- 	wiringPiSPIDataRW(channel,reg_data,1);
	//wpi- 	reg_data++;
	//wpi- }
	//wpi- #else
	//wpi- wiringPiSPIDataRW(channel,reg_data,len);
	//wpi- #endif

	//wpi- SPI_BME280_CS_High();

	if (debug) {
		printf("Read 0x%02X -> 0x", reg_addr & 0x7f);
		for (int i=0; i<len; i++)
			printf("%02X", reg_data[i]);
		printf("\n");
		fflush(stdout);
	}
	return rslt;
}

#define SPI_WRBUF_LEN 9 // 1 addr byte, up to 8 data bytes
int8_t user_spi_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
	int8_t rslt = 0;

	//wpi- SPI_BME280_CS_High();
	//wpi- SPI_BME280_CS_Low();

	//wpi- wiringPiSPIDataRW(channel,&reg_addr,1);
	if (len + 1 > SPI_WRBUF_LEN) {
		fprintf(stderr, "OOPS, cannot handle write > %d bytes. %d requested.\n",
			SPI_WRBUF_LEN-1, len);
		return 1;
	}

	uint8_t buf[SPI_WRBUF_LEN];
	buf[0] = reg_addr & 0x7f; // high bit = 0 for write
	memcpy(buf + 1, reg_data, len);
	linux_spi_write(buf,len+1);

	#if(USESPISINGLEREADWRITE)
	for(int i = 0; i < len ; i++)
	{
		wiringPiSPIDataRW(channel,reg_data,1);
		reg_data++;
	}
	#else
	//wpi- wiringPiSPIDataRW(channel,reg_data,len);
	#endif

	//wpi- SPI_BME280_CS_High();

	if (debug) {
		printf("Write 0x%02X <- 0x", reg_addr);
		for (int i=0; i<len; i++)
			printf("%02X", reg_data[i]);
		printf("\n");
		fflush(stdout);
	}
	return rslt;
}
#endif

void print_sensor_data(struct bme280_data *comp_data)
{
#ifdef BME280_FLOAT_ENABLE
	printf("temperature:%0.2f*C   pressure:%0.2fhPa   humidity:%0.2f%%\r\n",comp_data->temperature, comp_data->pressure/100, comp_data->humidity);
#else
	printf("temperature:%ld*C   pressure:%ldhPa   humidity:%ld%%\r\n",comp_data->temperature, comp_data->pressure/100, comp_data->humidity);
#endif
}

int8_t stream_sensor_data_forced_mode(struct bme280_dev *dev)
{
	int8_t rslt;
	uint8_t settings_sel;
	struct bme280_data comp_data;

	/* Recommended mode of operation: Indoor navigation */
	dev->settings.osr_h = BME280_OVERSAMPLING_1X;
	dev->settings.osr_p = BME280_OVERSAMPLING_16X;
	dev->settings.osr_t = BME280_OVERSAMPLING_2X;
	dev->settings.filter = BME280_FILTER_COEFF_16;

	settings_sel = BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL | BME280_FILTER_SEL;

	rslt = bme280_set_sensor_settings(settings_sel, dev);

	printf("Temperature           Pressure             Humidity\r\n");
	/* Continuously stream sensor data */
	while (1) {
		rslt = bme280_set_sensor_mode(BME280_FORCED_MODE, dev);
		/* Wait for the measurement to complete and print data @25Hz */
		dev->delay_ms(40);
		rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, dev);
		print_sensor_data(&comp_data);
	}
	return rslt;
}


int8_t stream_sensor_data_normal_mode(struct bme280_dev *dev)
{
	int8_t rslt;
	uint8_t settings_sel;
	struct bme280_data comp_data;

	/* Recommended mode of operation: Indoor navigation */
	dev->settings.osr_h = BME280_OVERSAMPLING_1X;
	dev->settings.osr_p = BME280_OVERSAMPLING_16X;
	dev->settings.osr_t = BME280_OVERSAMPLING_2X;
	dev->settings.filter = BME280_FILTER_COEFF_16;
	dev->settings.standby_time = BME280_STANDBY_TIME_62_5_MS;

	settings_sel = BME280_OSR_PRESS_SEL;
	settings_sel |= BME280_OSR_TEMP_SEL;
	settings_sel |= BME280_OSR_HUM_SEL;
	settings_sel |= BME280_STANDBY_SEL;
	settings_sel |= BME280_FILTER_SEL;
	rslt = bme280_set_sensor_settings(settings_sel, dev);
	rslt = bme280_set_sensor_mode(BME280_NORMAL_MODE, dev);

	printf("Temperature           Pressure             Humidity\r\n");
	while (1) {
		/* Delay while the sensor completes a measurement */
		dev->delay_ms(70);
		rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, dev);
		print_sensor_data(&comp_data);
	}

	return rslt;
}

#if(USEIIC)
int main(int argc, char* argv[])
{
	struct bme280_dev dev;
	int8_t rslt = BME280_OK;

	if ((fd = open(IIC_Dev, O_RDWR)) < 0) {
	printf("Failed to open the i2c bus %s", argv[1]);
	exit(1);
	}
	if (ioctl(fd, I2C_SLAVE, 0x77) < 0) {
	printf("Failed to acquire bus access and/or talk to slave.\n");
	exit(1);
	}
	dev.dev_id = BME280_I2C_ADDR_PRIM;//0x76
	//dev.dev_id = BME280_I2C_ADDR_SEC; //0x77
	dev.intf = BME280_I2C_INTF;
	dev.read = user_i2c_read;
	dev.write = user_i2c_write;
	dev.delay_ms = user_delay_ms;

	rslt = bme280_init(&dev);
	printf("\r\n BME280 Init Result is:%d \r\n",rslt);
	//stream_sensor_data_forced_mode(&dev);
	stream_sensor_data_normal_mode(&dev);
}
#else
void usage(char *name) {
	printf("Usage: %s [-d <spidev>] [-v]\n", name);
	printf("   -d <spidev>: SPI device (default %s)\n", spidev);
	printf("   -v         : verbose output (for debugging)\n");
}

int main(int argc, char* argv[])
{
	int c;
	while ((c = getopt(argc, argv, "d:vh")) != -1)
		switch (c)
		{
		case 'd':
			spidev = optarg;
			break;
		case 'h':
			usage(argv[0]);
			return 0;
		case 'v':
			debug = 1;
			break;
		default:
			usage(argv[0]);
			abort ();
		}

	/* wpi-
	if(wiringPiSetup() < 0)
	{
	return 1;
	} */

	// pinMode (27,OUTPUT) ;

	//wpi- SPI_BME280_CS_Low();//once pull down means use SPI Interface

	//wpi- wiringPiSPISetup(channel,2000000);
	spifd = open(spidev, O_RDWR);
	if (spifd < 0) {
		fprintf(stderr, "Error opening %s: %s\n", spidev, strerror(errno));
		return 1;
	}
	uint32_t speed = SPEED_HZ;
	int ret = ioctl(spifd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
	if (ret == -1) {
		fprintf(stderr, "Error setting write speed on %s: %s\n", spidev, strerror(errno));
		return 1;
	}
	ret = ioctl(spifd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
	if (ret == -1) {
		fprintf(stderr, "Error setting read speed on %s: %s\n", spidev, strerror(errno));
		return 1;
	}

	struct bme280_dev dev;
	int8_t rslt = BME280_OK;

	dev.dev_id = 0;
	dev.intf = BME280_SPI_INTF;
	dev.read = user_spi_read;
	dev.write = user_spi_write;
	dev.delay_ms = user_delay_ms;

	rslt = bme280_init(&dev);

	//uint8_t reg_addr = BME280_CONFIG_ADDR;
	//uint8_t reg_data;

	//rslt = bme280_get_regs(reg_addr, &reg_data, 1, &dev);
    //reg_data = (reg_data & 0xFE);// 4-wire SPI mode - force - TBD
    /* Write 4-wire SPI modes in the register */
	//rslt = bme280_set_regs(&reg_addr, &reg_data, 1, &dev);

	printf("\r\n BME280 Init Result is:%d \r\n",rslt);
	//stream_sensor_data_forced_mode(&dev);
	stream_sensor_data_normal_mode(&dev);

	close(spifd);
}
#endif
