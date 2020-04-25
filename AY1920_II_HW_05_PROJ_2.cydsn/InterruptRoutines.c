/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/
#include "InterruptRoutines.h"

CY_ISR(Custom_Timer_ISR){

    

    Timer_ReadStatusRegister(); // Read Timer Status Register in order to reset counter and trigger the ISR
    Start_reading_flag=1;

}
/* [] END OF FILE */
