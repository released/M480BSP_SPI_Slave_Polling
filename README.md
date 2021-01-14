# M480BSP_SPI_Slave_Polling
 M480BSP_SPI_Slave_Polling

update @ 2021/01/14

1. SPI hw config : PA0MFP_SPI0_MOSI , PA1MFP_SPI0_MISO , PA2MFP_SPI0_CLK , PA3MFP_SPI0_SS

2. SPI initial as SPI slave , receive and transmit with polling

3. under SPI_Slave_Process , there are 2 transmit selection 

send data at next clock in same packet

![image](https://github.com/released/M480BSP_SPI_Slave_Polling/blob/main/capture_1_01.jpg)

![image](https://github.com/released/M480BSP_SPI_Slave_Polling/blob/main/capture_1_02.jpg)

send data packet , at next transmission

![image](https://github.com/released/M480BSP_SPI_Slave_Polling/blob/main/capture_2_01.jpg)

![image](https://github.com/released/M480BSP_SPI_Slave_Polling/blob/main/capture_2_02.jpg)

