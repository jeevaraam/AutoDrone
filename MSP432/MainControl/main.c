#include "msp.h"
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

char message[10];

enum regCounts
{
    REG_COUNT_GPS=7,
    REG_COUNT_IMU=5
};

enum subSystemAddress
{
    SUBSYSTEM_COUNT = 2,
    SUBSYSTEM_ADDRESS_GPS_0 = 0x21,
    SUBSYSTEM_ADDRESS_IMU_0 = 0x11
};

const uint8_t subSystemRequestOrder[SUBSYSTEM_COUNT] = {SUBSYSTEM_ADDRESS_GPS_0,SUBSYSTEM_ADDRESS_IMU_0};
const uint8_t subSystemRegisterCount[SUBSYSTEM_COUNT] = {REG_COUNT_GPS,REG_COUNT_IMU};

enum gpsRegs
{
    GPS_SATS=1,
    GPS_LATITUDE=2,
    GPS_LONGITUDE=3,
    GPS_ALTITUDE=4,
    GPS_H_ACCURACY=5,
    GPS_V_ACCURACY=6,
    GPS_REFRESH_RATE=7
};

enum imuRegs
{
    IMU_SUBSYSTEM_TYPE=1,
    IMU_X_AXIS=2,
    IMU_Y_AXIS=3,
    IMU_Z_AXIS=4,
    IMU_REFRESH_RATE=5
};

const uint8_t subSystemRegisterOrder[10][10] = { {GPS_SATS,GPS_LATITUDE,GPS_LONGITUDE,GPS_ALTITUDE,GPS_H_ACCURACY,GPS_V_ACCURACY,GPS_REFRESH_RATE},
                                                 {IMU_SUBSYSTEM_TYPE,IMU_X_AXIS,IMU_Y_AXIS,IMU_Z_AXIS,IMU_REFRESH_RATE} };
const uint8_t subSystemRegisterSize[10][10] = { { 1,4,4,4,4,4,4}, {1,4,4,4,4} };

struct gpsDATA
{
    uint8_t sats_connected;
    float latitude;
    float longitude;
    float altitude;
    float h_accuracy;
    float v_accuracy;
    float refresh_rate;
};

struct gpsDATA gps;

struct imuData
{
    uint8_t subsystem_type;
    float xaxis;
    float yaxis;
    float zaxis;
    float refresh_rate;
};

struct imuData imu;

uint8_t RXData[4] = {0};
uint8_t RXDataPointer;
uint8_t i2CRequestSize = 0;

void GPIO_init(void)
{
  P1->DIR = 0xFF;
  P1->OUT = 0x00;
  P2->DIR = 0xFF;
  P2->OUT = 0x00;
  P3->DIR = 0xFF;
  P3->OUT = 0x00;
  P4->DIR = 0xFF;
  P4->OUT = 0x00;
  P5->DIR = 0xFF;
  P5->OUT = 0x00;
  P6->DIR = 0xFF;
  P6->OUT = 0x00;
  P7->DIR = 0xFF;
  P7->OUT = 0x00;
  P8->DIR = 0xFF;
  P8->OUT = 0x00;
  P9->DIR = 0xFF;
  P9->OUT = 0x00;
  P10->DIR = 0xFF;
  P10->OUT = 0x00;
}

void Clocks_init(void)
{
    uint32_t currentPowerState;
    currentPowerState = PCM->CTL0 & PCM_CTL0_CPM_MASK;

    if (!(currentPowerState != PCM_CTL0_CPM_0))
    {
        while ((PCM->CTL1 & PCM_CTL1_PMR_BUSY));
        PCM->CTL0 = PCM_CTL0_KEY_VAL | PCM_CTL0_AMR_1;
        while ((PCM->CTL1 & PCM_CTL1_PMR_BUSY));

        if (!(PCM->IFG & PCM_IFG_AM_INVALID_TR_IFG))
        {
            if (!((PCM->CTL0 & PCM_CTL0_CPM_MASK) != PCM_CTL0_CPM_1))
            {
                FLCTL->BANK0_RDCTL = (FLCTL->BANK0_RDCTL & ~(FLCTL_BANK0_RDCTL_WAIT_MASK)) |
                            FLCTL_BANK0_RDCTL_WAIT_1;
                    FLCTL->BANK1_RDCTL  = (FLCTL->BANK0_RDCTL & ~(FLCTL_BANK1_RDCTL_WAIT_MASK)) |
                            FLCTL_BANK1_RDCTL_WAIT_1;
                    CS->KEY = CS_KEY_VAL ;                  // Unlock CS module for register access
                    CS->CTL0 = 0;                           // Reset tuning parameters
                    CS->CTL0 = CS_CTL0_DCORSEL_5;           // Set DCO to 48MHz
                    /* Select MCLK = DCO, no divider */
                    CS->CTL1 = CS->CTL1 & ~(CS_CTL1_SELM_MASK | CS_CTL1_DIVM_MASK) |
                            CS_CTL1_SELM_3;
                    CS->CTL1 = CS->CTL1 & ~(CS_CTL1_DIVS_MASK) | CS_CTL1_DIVS_4;
                    CS->KEY = 0;
            }

        }

    }

}

void UART2_init(void)
{
    EUSCI_A0->CTLW0 |= 1;                               // put in reset mode for config
    EUSCI_A0->MCTLW = 0;                              //disable oversampling
    EUSCI_A0->CTLW0 = 0x0081;                      //1 stop bit, no parity, SMCLK, 8-bit data
    EUSCI_A0->BRW = 26;                                 //Set Baudrate 3,000,000 / 115200 = 26
    P1->SEL0 |= 0x0C;                                        //Config GPIO P3.3, P3.2 for UART
    P1->SEL1 &= ~0x0C;
    EUSCI_A0->CTLW0 &= ~1;                          //take UART out of reset mode
}

void I2C_init(void)
{
    P1->SEL0 |= BIT6 | BIT7;

    NVIC->IP[5] = (NVIC->IP[5]&0xFFFFFF00)|0x00000060;
    NVIC->ISER[0] = 1 << ((EUSCIB0_IRQn) & 31);

    EUSCI_B0->CTLW0 |= EUSCI_A_CTLW0_SWRST;
    EUSCI_B0->CTLW0 = EUSCI_A_CTLW0_SWRST | EUSCI_B_CTLW0_MODE_3 | EUSCI_B_CTLW0_MST | EUSCI_B_CTLW0_SYNC | EUSCI_B_CTLW0_SSEL__SMCLK;
    EUSCI_B0->CTLW1 |= EUSCI_B_CTLW1_ASTP_2;
    EUSCI_B0->BRW = 7;
    EUSCI_B0->TBCNT = 0x0001;
    EUSCI_B0->CTLW0 &= ~EUSCI_B_CTLW0_SWRST;

    EUSCI_B0->IE |= EUSCI_A_IE_RXIE | EUSCI_B_IE_NACKIE;

}

void I2C_Request(uint8_t slaveAddress, uint8_t registerAddress, uint8_t byteCount)
{
    EUSCI_B0->I2CSA = slaveAddress;

    while (EUSCI_B0->CTLW0 & EUSCI_B_CTLW0_TXSTP);

    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TR | EUSCI_B_CTLW0_TXSTT;

    EUSCI_B0->TXBUF = registerAddress;
    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTP;

    EUSCI_B0->TBCNT = byteCount;
    EUSCI_B0->CTLW0 &= ~EUSCI_B_CTLW0_TR;
    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTT;
}

float byteStreamToFloat(uint8_t * data)
{
    float *temp;
    temp = (float*)data;
    return *temp;
}

void TA2_0_IRQHandler(void)
{

    int i,j;

    for( i= 0; i < SUBSYSTEM_COUNT; i++)
    {
        for ( j = 0; j < subSystemRegisterCount[i]; j++)
        {
            i2CRequestSize = subSystemRegisterSize[i][j];
            I2C_Request(subSystemRequestOrder[i], subSystemRegisterOrder[i][j], i2CRequestSize);

            while (EUSCI_B0->CTLW0 & EUSCI_B_CTLW0_TXSTP);

            switch(subSystemRequestOrder[i])
            {
            case SUBSYSTEM_ADDRESS_GPS_0: switch(subSystemRegisterOrder[i][j])
                                          {
                                             case GPS_SATS: gps.sats_connected = RXData[0];
                                             break;

                                             case GPS_LATITUDE: gps.latitude = byteStreamToFloat(&RXData);
                                             break;

                                             case GPS_LONGITUDE: gps.longitude = byteStreamToFloat(&RXData);
                                             break;

                                             case GPS_ALTITUDE: gps.altitude = byteStreamToFloat(&RXData);
                                             break;

                                             case GPS_H_ACCURACY: gps.h_accuracy = byteStreamToFloat(&RXData);
                                             break;

                                             case GPS_V_ACCURACY: gps.v_accuracy = byteStreamToFloat(&RXData);
                                             break;

                                             case GPS_REFRESH_RATE: gps.refresh_rate = byteStreamToFloat(&RXData);
                                             break;
                                          }
            case SUBSYSTEM_ADDRESS_IMU_0 : switch(subSystemRegisterOrder[i][j])
                                           {
                                             case IMU_SUBSYSTEM_TYPE: imu.subsystem_type = RXData[0];
                                             break;

                                             case IMU_X_AXIS: imu.xaxis = byteStreamToFloat(&RXData);
                                             break;

                                             case IMU_Y_AXIS: imu.yaxis = byteStreamToFloat(&RXData);
                                             break;

                                             case IMU_Z_AXIS: imu.zaxis = byteStreamToFloat(&RXData);
                                             break;

                                             case IMU_REFRESH_RATE: imu.refresh_rate = byteStreamToFloat(&RXData);
                                             break;
                                           }
            }
        }
    }

    for (i = 0; i < 7; i++)
            {
                while(!(EUSCI_A0->IFG & 0x02)) { }                          // wait for transmit buffer empty
                EUSCI_A0->TXBUF = message[i];                             //send a char
            }
    TIMER_A2->CCTL[0] &= ~0x0001;
}

void EUSCIB0_IRQHandler(void)
{
    if (EUSCI_B0->IFG & EUSCI_B_IFG_NACKIFG)
    {
        EUSCI_B0->IFG &= ~ EUSCI_B_IFG_NACKIFG;

        // I2C start condition
        EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTT;
    }
    if (EUSCI_B0->IFG & EUSCI_B_IFG_RXIFG0)
    {
        EUSCI_B0->IFG &= ~ EUSCI_B_IFG_RXIFG0;

        // Get RX data
        RXData[RXDataPointer++] = EUSCI_B0->RXBUF;

        if (RXDataPointer >= i2CRequestSize)
        {
            RXDataPointer = 0;
        }
    }
}

void TimerA2_Init()
{
    TIMER_A2->CTL = 0x02C0;
    TIMER_A2->CCTL[0] = 0x0010;
    TIMER_A2->CCR[0] = 0xFFFF;
    TIMER_A2->EX0 = 0xE;
    NVIC->IP[3] = (NVIC->IP[3]&0xFFFFFF00)|0x00000040; // priority 2
    NVIC->ISER[0] = 0x00001000;
    TIMER_A2->CTL |= 0x0014;
}

int main(void)
{
    Clocks_init();
    GPIO_init();
    UART2_init();
    I2C_init();
    TimerA2_Init();

    MAP_Interrupt_enableSleepOnIsrExit();
    __enable_interrupts();

    while (1)
    {
        MAP_PCM_gotoLPM3();
    }
}
