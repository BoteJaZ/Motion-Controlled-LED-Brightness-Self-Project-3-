//#############################################################################
//
// To Dim an LED using motion control
// CPU1 controls ePWM1 and I2CA
// ePWM1 - sawtooth waveform of 10 kHz
// Used for I2CA module to get motion readings from MPU 6050 and updating CMPA accordingly, eventually varying the brightness of the LED
//
//#############################################################################

//
// Included Files
//
#include "F28x_Project.h"
#include "driverlib.h"
#include "device.h"

#define MPU6050_ADDR          0x68

#define MPU6050_WHO_AM_I      0x75
#define MPU6050_PWR_MGMT_1    0x6B
#define MPU6050_ACCEL_CONFIG  0x1C
#define MPU6050_ACCEL_XOUT_H  0x3B

//
// Main
//
uint8_t who;
uint8_t whoami;

uint8_t accel_raw[6];
int16_t ax, ay, az;
float ax_g, ay_g, az_g;

volatile uint16_t x = 2500; 

void i2c_WriteByte(uint16_t slaveaddr, uint16_t regaddr, uint16_t data);
void i2c_ReadByte(uint16_t slaveAddr, uint16_t regAddr, uint8_t *data);
void i2c_BurstReadByte(uint16_t slaveaddr, uint16_t regaddr, uint8_t *buffer, uint16_t count);
void I2CA_Init(void);
void mpu6050_init(void);

void main(void)
{
    // Initialize the system
    InitSysCtrl();

    // Set the frequency clock for ePWM module to less than or equal to 100 MHz
    EALLOW;
    ClkCfgRegs.PERCLKDIVSEL.bit.EPWMCLKDIV = 1;

    // Assign ePWM1 to CPU1 
    DevCfgRegs.CPUSEL0.bit.EPWM1 = 0;

    // Enable clock access to ePWM1 via CPU1
    CpuSysRegs.PCLKCR2.bit.EPWM1 = 1;

    // Assign I2CA to CPU1
    DevCfgRegs.CPUSEL7.bit.I2C_A = 0;

    // Enable clock access to I2CA via CPU1
    CpuSysRegs.PCLKCR9.bit.I2C_A = 1;
    EDIS;

    // Configure GPIO0 as ePWM functionality
    // GPIO0 Group 00b
    EALLOW;
    GpioCtrlRegs.GPAGMUX1.bit.GPIO0 = 0;

    // GPIO0 as ePWM pin
    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;
    
    // Configure GPIO104 and 105 as I2C pins
    // GPIO104 and 105 Group 00b
    GpioCtrlRegs.GPDGMUX1.bit.GPIO104 = 0;
    GpioCtrlRegs.GPDGMUX1.bit.GPIO105 = 0;

    // GPIO104 and 105 as SDA A and SCL A respectively
    GpioCtrlRegs.GPDMUX1.bit.GPIO104 = 1;
    GpioCtrlRegs.GPDMUX1.bit.GPIO105 = 1;

    // GPIO104 and 105 as Open drain
    GpioCtrlRegs.GPDODR.bit.GPIO104 = 1;
    GpioCtrlRegs.GPDODR.bit.GPIO105 = 1;

    // Enable GPIO104 and 105 pullups 
    GpioCtrlRegs.GPDPUD.bit.GPIO104 = 0;
    GpioCtrlRegs.GPDPUD.bit.GPIO105 = 0;

    /* SDA = GPIO104 */
    GpioCtrlRegs.GPDQSEL1.bit.GPIO104 = 3;  // ASYNC

    /* SCL = GPIO105 */
    GpioCtrlRegs.GPDQSEL1.bit.GPIO105 = 3;  // ASYNC

    EDIS;


    // Stop timer clock for ePWM1
    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;
    EDIS;

    // Configure the ePWM1 module
    // Time base sub-module
    // Clock fed to ePWM module is 100 MHz. TBCLK = 100 MHz/ (HSPCLKDIV * CLKDIV) = 100 MHz/ (2 * 1) = 50 MHz
    // Configure ePWM1
    EPwm1Regs.TBCTL.bit.FREE_SOFT = 2;
    EPwm1Regs.TBCTL.bit.CLKDIV = 0;
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = 1;
    EPwm1Regs.TBCTL.bit.PRDLD = 0;
    EPwm1Regs.TBCTL.bit.CTRMODE = 0;        // up count mode

    // Load value in Period Register
    EPwm1Regs.TBPRD = 5000 - 1;             // 10kHz sawtooth waveform

    // Counter-compare sub-module 
    EPwm1Regs.CMPCTL.bit.LOADAMODE = 0;
    EPwm1Regs.CMPCTL.bit.SHDWAMODE = 0;
    

    // Action Qualifier sub-module
    // TBCTR = 0, A will be high (bit ZRO - AQCTLA/B)
    EPwm1Regs.AQCTLA.bit.ZRO = 2;
    EPwm1Regs.AQCTLA.bit.CAU = 1;

    // Configure the I2CA module

    // Set slave address
    I2caRegs.I2CSAR.all = MPU6050_ADDR;

    // Come out of reset
    I2caRegs.I2CMDR.bit.IRS = 1;

    I2CA_Init();

    // Initialize MPU 6050
    mpu6050_init();

    

    // Start the timer counter for ePWM 1
    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;
    EDIS;   
     
    // Infinite loop
    while (1) 
    {
        // i2c_BurstReadByte(MPU6050_ADDR, MPU6050_ACCEL_XOUT_H, accel_raw, 6);

        // ax = (accel_raw[0] << 8) | accel_raw[1];
        // ay = (accel_raw[2] << 8) | accel_raw[3];
        // az = (accel_raw[4] << 8) | accel_raw[5];

        // ax_g = ax / 8192.0f;
        // ay_g = ay / 8192.0f;
        // az_g = az / 8192.0f;
        // i2c_ReadByte(0x68, MPU6050_WHO_AM_I, &who);
        // whoami = who;
        i2c_ReadByte(0x68, 0x75, &who);  // WHO_AM_I
        // who MUST be 0x68
        DELAY_US(100000);
    }
}

void i2c_WriteByte(uint16_t slaveaddr, uint16_t regaddr, uint16_t data)
{
    // Loop until Bus is free
    while (I2caRegs.I2CSTR.bit.BB != 0) {}  

    // Set data count 
    I2caRegs.I2CCNT = 2;   

    // Set I2C module as master transmitter in non-repeat mode
    I2caRegs.I2CMDR.all = 0x6620U;

    // transmit the data
    I2caRegs.I2CDXR.bit.DATA = data;

    // Wait until data is transmitted
    while (I2caRegs.I2CSTR.bit.XRDY != 1) {}

    // generate Stop condition
    I2caRegs.I2CMDR.bit.STP = 1;
}

void I2CA_Init(void)
{
    // Reset the I2CA module 
    I2caRegs.I2CMDR.bit.IRS = 0;
    
    // Set prescale value such that module clock is 10 MHz
    I2caRegs.I2CPSC.all = 19;

    // Set ICCL and ICCH such that Fmst = 100 KHz
    I2caRegs.I2CCLKL = 45;
    I2caRegs.I2CCLKH = 45;    

    // Set slave to whom we want to communicate
    I2caRegs.I2CSAR.bit.SAR = MPU6050_ADDR;  

    // Reset the I2CA module 
    I2caRegs.I2CMDR.bit.IRS = 1;
}

// void i2c_WriteByte(uint16_t slaveAddr, uint16_t regAddr, uint16_t data)
// {
//     /* Wait for previous STOP */
//     while (I2caRegs.I2CMDR.bit.STP);

//     /* Wait until bus is free */
//     while (I2caRegs.I2CSTR.bit.BB);

//     /* Slave address */
//     I2caRegs.I2CSAR.all = slaveAddr;

//     /* 2 bytes: reg + data */
//     I2caRegs.I2CCNT = 2;

//     /* Load TX data */
//     I2caRegs.I2CDXR.all = regAddr;
//     I2caRegs.I2CDXR.all = data;

//     /* Master transmit, START + STOP */
//     I2caRegs.I2CMDR.all = 0x6E20;
//     // MST=1, TRX=1, STT=1, STP=1, IRS=1

//     /* Wait for STOP */
//     while (I2caRegs.I2CSTR.bit.SCD == 0);
//     I2caRegs.I2CSTR.bit.SCD = 1;
// }




// void i2c_WriteByte(uint16_t slaveaddr, uint16_t regaddr, uint16_t data)
// {
//     //Wait until the STP bit is cleared from  any previous master communication
//     while (I2caRegs.I2CMDR.bit.STP != 0) {}

//     // Set slave to whom we want to communicate
//     I2caRegs.I2CSAR.all = slaveaddr;  

//     // Loop until Bus is free
//     while (I2caRegs.I2CSTR.bit.BB != 0) {} 

//     // Set data count 
//     I2caRegs.I2CCNT = 2;     

//     // Set I2C module as master transmitter
//     I2caRegs.I2CMDR.all = 0x6E20;

// }


// void i2c_BurstReadByte(uint16_t slaveaddr, uint16_t regaddr, uint8_t *buffer, uint16_t count)
// {
//     uint16_t i; 

//     // Loop until Bus is free
//     while (I2caRegs.I2CSTR.bit.BB != 0) {}      

//     // Set slave to whom we want to communicate
//     I2caRegs.I2CSAR.bit.SAR = slaveaddr;        

//     // Set I2C module as transmitter
//     I2caRegs.I2CMDR.bit.TRX = 1;

//     // Set data count 
//     I2caRegs.I2CCNT = 1;

//     // generate Start condition
//     I2caRegs.I2CMDR.bit.STT = 1;

//     // Wait until transmitter is done transmitting previous data
//     while (I2caRegs.I2CSTR.bit.XRDY != 1) {}

//     // transmit register address on slave where we want to write the data
//     I2caRegs.I2CDXR.bit.DATA = regaddr;

//     while (!I2caRegs.I2CSTR.bit.ARDY);

//     // Set I2C module as receiver
//     I2caRegs.I2CMDR.bit.TRX = 0;

//     // Set data count 
//     I2caRegs.I2CCNT = count;

    

//     // generate Repeated start condition
//     I2caRegs.I2CMDR.bit.STT = 1;

//     for (i = 0; i < count; i++)
//     {
//         while (I2caRegs.I2CSTR.bit.RRDY == 0) {}
//         buffer[i] = (uint8_t)I2caRegs.I2CDRR.bit.DATA;
//     }

//     // generate Stop condition
//     I2caRegs.I2CMDR.bit.STP = 1;

//     // Wait for STOP to complete
//     while (I2caRegs.I2CSTR.bit.SCD != 1) {}
//     I2caRegs.I2CSTR.bit.SCD = 1;
// }

void i2c_ReadByte(uint16_t slaveAddr, uint16_t regAddr, uint8_t *data)
{
    // Loop until Bus is free
    while (I2caRegs.I2CSTR.bit.BB != 0) {}    

    

    // Set data count 
    I2caRegs.I2CCNT = 1;  

    // Set I2C module as master transmitter in non-repeat mode
    I2caRegs.I2CMDR.all = 0x6620U;

    // transmit the data
    I2caRegs.I2CDXR.bit.DATA = regAddr;

    /* Wait until register address sent */
    while (I2caRegs.I2CSTR.bit.ARDY != 1) {}

    // Set data count 
    I2caRegs.I2CCNT = 1;

    // Set I2C module as master receiver in non-repeat mode
    I2caRegs.I2CMDR.all = 0x6420U;

    // Wait for data 
    while (I2caRegs.I2CSTR.bit.RRDY != 1) {}

    *data = I2caRegs.I2CDRR.all & 0xFF;

    // generate Stop condition
    I2caRegs.I2CMDR.bit.STP = 1;
}



// void i2c_ReadByte(uint16_t slaveaddr, uint16_t regaddr, uint8_t *data)
// {
//     // Wait until the STP bit is cleared from  any previous master communication
//     while (I2caRegs.I2CMDR.bit.STP != 0) {}

//     // Set slave to whom we want to communicate
//     I2caRegs.I2CSAR.all = slaveaddr; 

//     // Loop until Bus is free
//     while (I2caRegs.I2CSTR.bit.BB != 0) {} 

//     // Set data count 
//     I2caRegs.I2CCNT = 1;

//     // Send slave address via I2C module as master transmitter
//     I2caRegs.I2CMDR.all = 0x2620;

//     // Send slave address via I2C module as master transmitter
//     I2caRegs.I2CMDR.all = 0x2C20;

//     *data = I2caRegs.I2CDRR.all & 0xFF;
// }

// void i2c_BurstReadByte(uint16_t slaveaddr,
//                        uint16_t regaddr,
//                        uint8_t *buffer,
//                        uint16_t count)
// {
//     uint16_t i;

//     /* Wait until bus is free */
//     while (I2caRegs.I2CSTR.bit.BB == 1);

//     /* Clear status flags */
//     I2caRegs.I2CSTR.all = 0x0000;

//     /* Set slave address */
//     I2caRegs.I2CSAR.all = slaveaddr;

//     /* TRANSMIT register address */
//     I2caRegs.I2CCNT = 1;
//     I2caRegs.I2CMDR.all = 0x2620;  
//     // MST=1, TRX=1, STT=1, FREE=1

//     /* Wait for TX ready */
//     while (I2caRegs.I2CSTR.bit.XRDY == 0);
//     I2caRegs.I2CDXR.all = regaddr;

//     /* Wait for address phase complete */
//     while (I2caRegs.I2CSTR.bit.ARDY == 0);

//     /* RECEIVE data */
//     I2caRegs.I2CCNT = count;
//     I2caRegs.I2CMDR.all = 0x2420;
//     // MST=1, TRX=0, STT=1, FREE=1

//     for (i = 0; i < count; i++)
//     {
//         while (I2caRegs.I2CSTR.bit.RRDY == 0);
//         buffer[i] = I2caRegs.I2CDRR.all & 0xFF;
//     }

//     /* STOP */
//     I2caRegs.I2CMDR.bit.STP = 1;
//     while (I2caRegs.I2CSTR.bit.SCD == 0);
//     I2caRegs.I2CSTR.bit.SCD = 1;
// }

void mpu6050_init(void)
{
    uint8_t buffer[1]; 

    i2c_ReadByte(0x68, 0x75, &who);  // WHO_AM_I
    // who MUST be 0x68
    DELAY_US(100000);

    // Wake up 
    i2c_WriteByte(MPU6050_ADDR, MPU6050_PWR_MGMT_1, 0x00);
    DELAY_US(100000);       // 100 ms

    // Reset Accelerometer
    i2c_WriteByte(MPU6050_ADDR, MPU6050_PWR_MGMT_1, 0x80);
    DELAY_US(150000);       // 150 ms

    // Set data format range to +-4g
    i2c_WriteByte(MPU6050_ADDR, MPU6050_ACCEL_CONFIG, 0x08);

    // Read the Device ID, this should return 0x68
    // i2c_BurstReadByte(MPU6050_ADDR, MPU6050_WHO_AM_I, &buffer, 1);
    // whoami = buffer[0];


}
//
// End of File
//
