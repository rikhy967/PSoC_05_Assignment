/**
* \brief Main source file for the I2C-Master project.
*
* In this project we set up a I2C master device with
* to understand the I2C protocol and communicate with a
* a I2C Slave device (LIS3DH Accelerometer).
*
* \author Riccardo Levi
* \date , 2020
*/

// Include required header files
#include "I2C_Interface.h"
#include "InterruptRoutines.h"
#include "project.h"
#include "stdio.h"

/**
*   \brief 7-bit I2C address of the slave device.
*/
#define LIS3DH_DEVICE_ADDRESS 0x18

/**
*   \brief Address of the WHO AM I register
*/
#define LIS3DH_WHO_AM_I_REG_ADDR 0x0F

/**
*   \brief Address of the Status register
*/
#define LIS3DH_STATUS_REG 0x27
#define LIS3DH_STATUS_REG_NEW_VALUES 0x07

/**
*   \brief Address of the Control register 1
*/
#define LIS3DH_CTRL_REG1 0x20

/**
*   \brief Hex value to set normal mode 50Hz to the accelerator
*/
#define LIS3DH_50Hz_NORMAL_MODE_CTRL_REG1 0x47

/**
*   \brief Hex value to set normal mode or high resolution mode  100Hz to the accelerator
*/
#define LIS3DH_100Hz_CTRL_REG1 0x57
/**
*   \brief  Address of the Temperature Sensor Configuration register
*/
#define LIS3DH_TEMP_CFG_REG 0x1F

#define LIS3DH_TEMP_CFG_REG_ACTIVE 0xC0
#define LIS3DH_TEMP_CFG_REG_NOT_ACTIVE 0x00 //Disable Temperature sensor reading

/**
*   \brief Address of the Control register 4
*/
#define LIS3DH_CTRL_REG4 0x23


#define LIS3DH_CTRL_REG4_2G_NORMAL 0x00 // ± 2g FSR Normal Mode
#define LIS3DH_CTRL_REG4_4G_HIGH 0x18 // ± 4g FSR High Resolution Mode

/**
*   \brief Address of the ADC output LSB register
*/
#define LIS3DH_OUT_ADC_3L 0x0C

/**
*   \brief Address of the ADC output MSB register
*/
#define LIS3DH_OUT_ADC_3H 0x0D

/**
*   \brief Address of the Accelerometer output LSB register
*/
#define LIS3DH_OUT_X_L 0x28
#define LIS3DH_OUT_Y_L 0x2A
#define LIS3DH_OUT_Z_L 0x2C
/**
*   \brief Address of the Accelerometer output MSB register
*/
#define LIS3DH_OUT_X_H 0x29
#define LIS3DH_OUT_Y_H 0x2B
#define LIS3DH_OUT_Z_H 0x2D

/*
*  Sensitivity Level
*/

#define LIS3DH_SENS_2G 4 //Sensitivity for ± 2g FSR Normal Mode (mg/digit)
#define LIS3DH_SENS_4G 2 //Sensitivity for ± 2g FSR Normal Mode (mg/digit)
#define LIS3DH_RES_NORMAL 10 //Number of bits in Normal Mode Resolution



int main(void)
{
    CyGlobalIntEnable; /* Enable global interrupts. */

    /* Place your initialization/startup code here (e.g. MyInst_Start()) */
    I2C_Peripheral_Start();
    UART_Debug_Start();
    Timer_Start();
    isr_Timer_StartEx(Custom_Timer_ISR);
    
    CyDelay(5); //"The boot procedure is complete about 5 milliseconds after device power-up."
    
    // String to print out messages on the UART
    char message[50];

    // Check which devices are present on the I2C bus
    for (int i = 0 ; i < 128; i++)
    {
        if (I2C_Peripheral_IsDeviceConnected(i))
        {
            // print out the address is hex format
            sprintf(message, "Device 0x%02X is connected\r\n", i);
            UART_Debug_PutString(message); 
        }
        
    }
    
    /******************************************/
    /*            I2C Reading                 */
    /******************************************/
    
    /* Read WHO AM I REGISTER register */
    uint8_t who_am_i_reg;
    ErrorCode error = I2C_Peripheral_ReadRegister(LIS3DH_DEVICE_ADDRESS,
                                                  LIS3DH_WHO_AM_I_REG_ADDR, 
                                                  &who_am_i_reg);
    if (error == NO_ERROR)
    {
        sprintf(message, "WHO AM I REG: 0x%02X [Expected: 0x33]\r\n", who_am_i_reg);
        UART_Debug_PutString(message); 
    }
    else
    {
        UART_Debug_PutString("Error occurred during I2C comm\r\n");   
    }
    
    /*      I2C Reading Status Register       */
    
    uint8_t status_register; 
    error = I2C_Peripheral_ReadRegister(LIS3DH_DEVICE_ADDRESS,
                                        LIS3DH_STATUS_REG,
                                        &status_register);
    
    if (error == NO_ERROR)
    {
        sprintf(message, "STATUS REGISTER: 0x%02X\r\n", status_register);
        UART_Debug_PutString(message); 
    }
    else
    {
        UART_Debug_PutString("Error occurred during I2C comm to read status register\r\n");   
    }
    
    /******************************************/
    /*        Read Control Register 1         */
    /******************************************/
    uint8_t ctrl_reg1; 
    error = I2C_Peripheral_ReadRegister(LIS3DH_DEVICE_ADDRESS,
                                        LIS3DH_CTRL_REG1,
                                        &ctrl_reg1);
    
    if (error == NO_ERROR)
    {
        sprintf(message, "CONTROL REGISTER 1: 0x%02X\r\n", ctrl_reg1);
        UART_Debug_PutString(message); 
    }
    else
    {
        UART_Debug_PutString("Error occurred during I2C comm to read control register 1\r\n");   
    }
    
    /******************************************/
    /*            I2C Writing                 */
    /******************************************/
    
        
    UART_Debug_PutString("\r\nWriting new values..\r\n");
    
    if (ctrl_reg1 != LIS3DH_100Hz_CTRL_REG1)
    {
        ctrl_reg1 = LIS3DH_100Hz_CTRL_REG1;
    
        error = I2C_Peripheral_WriteRegister(LIS3DH_DEVICE_ADDRESS,
                                             LIS3DH_CTRL_REG1,
                                             ctrl_reg1);
    
        if (error == NO_ERROR)
        {
            sprintf(message, "CONTROL REGISTER 1 successfully written as: 0x%02X\r\n", ctrl_reg1);
            UART_Debug_PutString(message); 
        }
        else
        {
            UART_Debug_PutString("Error occurred during I2C comm to set control register 1\r\n");   
        }
    }
    
    /******************************************/
    /*     Read Control Register 1 again      */
    /******************************************/

    error = I2C_Peripheral_ReadRegister(LIS3DH_DEVICE_ADDRESS,
                                        LIS3DH_CTRL_REG1,
                                        &ctrl_reg1);
    
    if (error == NO_ERROR)
    {
        sprintf(message, "CONTROL REGISTER 1 after overwrite operation: 0x%02X\r\n", ctrl_reg1);
        UART_Debug_PutString(message); 
    }
    else
    {
        UART_Debug_PutString("Error occurred during I2C comm to read control register 1\r\n");   
    }
    
     /******************************************/
     /* I2C Reading Temperature sensor CFG reg */
     /******************************************/

    uint8_t tmp_cfg_reg;

    error = I2C_Peripheral_ReadRegister(LIS3DH_DEVICE_ADDRESS,
                                        LIS3DH_TEMP_CFG_REG,
                                        &tmp_cfg_reg);
    
    if (error == NO_ERROR)
    {
        sprintf(message, "TEMPERATURE CONFIG REGISTER: 0x%02X\r\n", tmp_cfg_reg);
        UART_Debug_PutString(message); 
    }
    else
    {
        UART_Debug_PutString("Error occurred during I2C comm to read temperature config register\r\n");   
    }
    
    
    tmp_cfg_reg = LIS3DH_TEMP_CFG_REG_NOT_ACTIVE; //Disable Temperature sensor reading
    
    error = I2C_Peripheral_WriteRegister(LIS3DH_DEVICE_ADDRESS,
                                         LIS3DH_TEMP_CFG_REG,
                                         tmp_cfg_reg);
    
    error = I2C_Peripheral_ReadRegister(LIS3DH_DEVICE_ADDRESS,
                                        LIS3DH_TEMP_CFG_REG,
                                        &tmp_cfg_reg);
    
    
    if (error == NO_ERROR)
    {
        sprintf(message, "TEMPERATURE CONFIG REGISTER after being updated: 0x%02X\r\n", tmp_cfg_reg);
        UART_Debug_PutString(message); 
    }
    else
    {
        UART_Debug_PutString("Error occurred during I2C comm to read temperature config register\r\n");   
    }
    
    uint8_t ctrl_reg4;

    error = I2C_Peripheral_ReadRegister(LIS3DH_DEVICE_ADDRESS,
                                        LIS3DH_CTRL_REG4,
                                        &ctrl_reg4);
    
    if (error == NO_ERROR)
    {
        sprintf(message, "CONTROL REGISTER 4: 0x%02X\r\n", ctrl_reg4);
        UART_Debug_PutString(message); 
    }
    else
    {
        UART_Debug_PutString("Error occurred during I2C comm to read control register4\r\n");   
    }
    
    
    ctrl_reg4 = LIS3DH_CTRL_REG4_4G_HIGH; // must be changed to the appropriate value
    
    error = I2C_Peripheral_WriteRegister(LIS3DH_DEVICE_ADDRESS,
                                         LIS3DH_CTRL_REG4,
                                         ctrl_reg4);
    
    error = I2C_Peripheral_ReadRegister(LIS3DH_DEVICE_ADDRESS,
                                        LIS3DH_CTRL_REG4,
                                        &ctrl_reg4);
    
    
    if (error == NO_ERROR)
    {
        sprintf(message, "CONTROL REGISTER 4 after being updated: 0x%02X\r\n", ctrl_reg4);
        UART_Debug_PutString(message); 
    }
    else
    {
        UART_Debug_PutString("Error occurred during I2C comm to read control register4\r\n");   
    }
    
    int16_t OutTemp;
    float32 OutTempHR_float;
    int32 OutTempHR_int;
    
 
    uint8_t header = 0xA0;
    uint8_t footer = 0xC0;
    uint8_t OutArray[8]; // Send an array that contains 2 byte per axis plus header and tail
    uint8_t OutArrayHR[14]; // Send an array that contains 2 byte per axis plus header and tail
    uint8_t AccelerometerData[2]; // Array that contains temporal data of each axis
    uint8_t Check_data; // Data read by the Status Register
    CYBIT flag_start_reading=0; // Flag used to start reading or not data from axis' registers
 
    
    
    
    OutArray[0] = header;
    OutArray[7] = footer;
    OutArrayHR[0] = header;
    OutArrayHR[13] = footer; 
    Start_reading_flag=0;

    
    /*for(;;)
    {
        
        // Check if new data is available by check the status register
        error = I2C_Peripheral_ReadRegister(LIS3DH_DEVICE_ADDRESS,
                                            LIS3DH_STATUS_REG,
                                            &Check_data);
        if(error == NO_ERROR)
        {
            //Check bit a bit with data read from the Status Register
            if ((Check_data&LIS3DH_STATUS_REG_NEW_VALUES)==LIS3DH_STATUS_REG_NEW_VALUES){
                flag_start_reading =1;
            }
            
        }
        
        if (flag_start_reading & Start_reading_flag){
        // Read X axis
        error = I2C_Peripheral_ReadRegisterMulti(LIS3DH_DEVICE_ADDRESS,
                                            LIS3DH_OUT_X_L,
                                            2,
                                            AccelerometerData);
        if(error == NO_ERROR)
        {
            OutTemp = (int16)((AccelerometerData[1] | (AccelerometerData[0]<<8)))>>4;
            OutTemp = OutTemp*LIS3DH_SENS_4G;
            OutTemp = OutTemp*9.81;
            OutArray[1] = (uint8_t)(OutTemp & 0xFF);
            OutArray[2] = (uint8_t)(OutTemp >> 8);

            
            
            
        }
        // Read Y axis
        error = I2C_Peripheral_ReadRegisterMulti(LIS3DH_DEVICE_ADDRESS,
                                            LIS3DH_OUT_Y_L,
                                            2,
                                            AccelerometerData);
        if(error == NO_ERROR)
        {
            OutTemp = (int16)((AccelerometerData[1] | (AccelerometerData[0]<<8)))>>4;
            OutTemp = OutTemp*LIS3DH_SENS_4G;
            OutTemp = OutTemp*9.81;
            OutArray[3] = (uint8_t)(OutTemp & 0xFF);
            OutArray[4] = (uint8_t)(OutTemp >> 8);

            
        }
        // Read Z axis
        error = I2C_Peripheral_ReadRegisterMulti(LIS3DH_DEVICE_ADDRESS,
                                            LIS3DH_OUT_Z_L,
                                            2,
                                            AccelerometerData);
        if(error == NO_ERROR)
        {
            OutTemp = (int16)((AccelerometerData[1] | (AccelerometerData[0]<<8)))>>4;
            OutTemp = OutTemp*LIS3DH_SENS_4G;
            OutTemp = OutTemp*9.81;
            OutArray[5] = (uint8_t)(OutTemp & 0xFF);
            OutArray[6] = (uint8_t)(OutTemp >> 8);

        }
        
        // Send all the measurements throught UART communication
        UART_Debug_PutArray(OutArray, 8);

        }
        flag_start_reading=0; // Reset flag checking LIS3DH Status Register
        Start_reading_flag=0; // Reset flag checking Timer Status Register
        
    }
    */
    for(;;)
    {
        
        // Check if new data is available by check the status register
        error = I2C_Peripheral_ReadRegister(LIS3DH_DEVICE_ADDRESS,
                                            LIS3DH_STATUS_REG,
                                            &Check_data);
        if(error == NO_ERROR)
        {
            //Check bit a bit with data read from the Status Register
            if ((Check_data&LIS3DH_STATUS_REG_NEW_VALUES)==LIS3DH_STATUS_REG_NEW_VALUES){
                flag_start_reading =1;
            }
            
        }
        
        if (flag_start_reading & Start_reading_flag){
        // Read X axis
        error = I2C_Peripheral_ReadRegisterMulti(LIS3DH_DEVICE_ADDRESS,
                                            LIS3DH_OUT_X_L,
                                            2,
                                            AccelerometerData);
        if(error == NO_ERROR)
        {
            OutTemp   = (int16)((AccelerometerData[1] | (AccelerometerData[0]<<8)))>>4;
            OutTemp = OutTemp*LIS3DH_SENS_4G;
            OutTempHR_float = OutTemp*9.80665;
            OutTempHR_int = (int32) OutTempHR_float;
            OutArrayHR[1] = (uint8_t)(OutTemp & 0xFF);
            OutArrayHR[2] = (uint8_t)((OutTemp >> 8)&0xFF);
            OutArrayHR[3] = (uint8_t)((OutTemp >> 16)&0xFF);
            OutArrayHR[4] = (uint8_t)(OutTemp >> 24);

            
            
            
        }
        // Read Y axis
        error = I2C_Peripheral_ReadRegisterMulti(LIS3DH_DEVICE_ADDRESS,
                                            LIS3DH_OUT_Y_L,
                                            2,
                                            AccelerometerData);
        if(error == NO_ERROR)
        {
            OutTemp = (int16)((AccelerometerData[1] | (AccelerometerData[0]<<8)))>>4;
            OutTemp = OutTemp*LIS3DH_SENS_4G;
            OutTempHR_float = (OutTemp)*9.80665;
            OutTempHR_int = (int32) OutTempHR_float;
            OutArrayHR[5] = (uint8_t)(OutTemp & 0xFF);
            OutArrayHR[6] = (uint8_t)((OutTemp >> 8)&0xFF);
            OutArrayHR[7] = (uint8_t)((OutTemp >> 16)&0xFF);
            OutArrayHR[8] = (uint8_t)(OutTemp >> 24);

            
        }
        // Read Z axis
        error = I2C_Peripheral_ReadRegisterMulti(LIS3DH_DEVICE_ADDRESS,
                                            LIS3DH_OUT_Z_L,
                                            2,
                                            AccelerometerData);
        if(error == NO_ERROR)
        {
            OutTemp = (int16)((AccelerometerData[1] | (AccelerometerData[0]<<8)))>>4;
            OutTemp = OutTemp*LIS3DH_SENS_4G;
            OutTempHR_float = OutTemp*9.80665;
            OutTempHR_int = (int32) OutTempHR_float;
            OutArrayHR[9] = (uint8_t)(OutTemp & 0xFF);
            OutArrayHR[10] = (uint8_t)((OutTemp >> 8)&0xFF);
            OutArrayHR[11] = (uint8_t)((OutTemp >> 16)&0xFF);
            OutArrayHR[12] = (uint8_t)(OutTemp >> 24);

        }
        
        // Send all the measurements throught UART communication
        UART_Debug_PutArray(OutArrayHR, 14);

        }
        flag_start_reading=0; // Reset flag checking LIS3DH Status Register
        Start_reading_flag=0; // Reset flag checking Timer Status Register
        
    }
}

/* [] END OF FILE */