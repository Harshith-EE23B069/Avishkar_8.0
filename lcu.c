#define IDLE_STATE     0
#define ACTIVE_STATE   1
#define CHARGING_STATE 2
#define ON_STATE       3
#define FAULT_STATE    4

#include <stdint.h>
#include <stddef.h>
#include <math.h>

#include <semaphore.h>
#include <unistd.h>
#include <pthread.h>
#include <errno.h>

#include <ti/display/Display.h>
#include <ti/drivers/Timer.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/UART.h>
#include <ti/display/DisplayUart.h>
#include <ti/drivers/CAN.h>
#include <ti/drivers/ADCBuf.h>

#include "ti_drivers_config.h"

Display_Handle display;
UART_Handle uartRF;
UART_Handle uartRB;
UART_Handle uartLB;
UART_Handle uartLF;
static CAN_Handle can;

void ConfigureDisplay(void);
void ConfigureTimer(void);
void ConfigureUART(void);
void ConfigureCAN(void);

void *CANThread(void *arg0);
void *faultThread(void *arg0);

uint8_t faultCheck(void);
float parseData(float offset, float factor, int len, int s_bit);

CAN_Frame readCAN = {0};

CAN_Frame LCU_Parameters_Acknowledgement = {0};
CAN_Frame LCU_Fault_Message = {0};

#define LCU_Sensors_RF_id                  0x001178FF
#define LCU_Sensors_RB_id                  0x001174FF
#define LCU_Sensors_LB_id                  0x001172FF
#define LCU_Sensors_LF_id                  0x001171FF

#define LCU_Temperature_RF_id              0x004178FF
#define LCU_Temperature_RB_id              0x004174FF
#define LCU_Temperature_LB_id              0x004172FF
#define LCU_Temperature_LF_id              0x004171FF

#define LCU_Parameters_Acknowledgement_id  0x001170FF
#define LCU_Fault_Message                  0x000370FF

#define LCU_Parameters_id                  0x0009F0FF
#define MCU_State_Fault_id                 0x00019001
#define MCU_Control_id                     0x000490FF

#define TEMPERATURE_OFFEST (0)
#define TEMPERATURE_FACTOR (1000)
#define TEMPERATURE_MIN    (0)
#define TEMPERATURE_MAX    (1000)

#define AIRGAP_OFFEST      (0)
#define AIRGAP_FACTOR      (1000)
#define AIRGAP_MIN         (0)
#define AIRGAP_MAX         (1000)

#define VOLTAGE_OFFSET     (0)
#define VOLTAGE_FACTOR     (1000)
#define VOLTAGE_MIN        (0)
#define VOLTAGE_MAX        (1000)

#define CURRENT_OFFSET     (0)
#define CURRENT_FACTOR     (1000)
#define CURRENT_MIN        (0)
#define CURRENT_MAX        (1000)

#define CONFIG_GPIO_ON  (1)
#define CONFIG_GPIO_OFF (0)

#define THREADSTACKSIZE (2048)

uint8_t LCU_state = IDLE_STATE;
uint64_t msg = 0;

bool Levitate_toggle = 0;
bool HEMS_LEMS_Precharge_Discharge = 0;

float Lateral_airgap_RF = 0.0;
float Lateral_airgap_RB = 0.0;
float Lateral_airgap_LB = 0.0;
float Lateral_airgap_LF = 0.0;

float Vertical_airgap_RF = 0.0;
float Vertical_airgap_RB = 0.0;
float Vertical_airgap_LB = 0.0;
float Vertical_airgap_LF = 0.0;

float HEMS_current_RF = 0.0;
float HEMS_current_RB = 0.0;
float HEMS_current_LB = 0.0;
float HEMS_current_LF = 0.0;

float LEMS_current_RF = 0.0;
float LEMS_current_RB = 0.0;
float LEMS_current_LB = 0.0;
float LEMS_current_LF = 0.0;

float HEMS_voltage_RF = 0.0;
float HEMS_voltage_RB = 0.0;
float HEMS_voltage_LB = 0.0;
float HEMS_voltage_LF = 0.0;

float LEMS_voltage_RF = 0.0;
float LEMS_voltage_RB = 0.0;
float LEMS_voltage_LB = 0.0;
float LEMS_voltage_LF = 0.0;
float LIM_current_LF = 0.0;

float Thermistor_1_RF = 0.0;
float Thermistor_2_RF = 0.0;
float Thermistor_3_RF = 0.0;
float Thermistor_4_RF = 0.0;

float Thermistor_1_RB = 0.0;
float Thermistor_2_RB = 0.0;
float Thermistor_3_RB = 0.0;
float Thermistor_4_RB = 0.0;

float Thermistor_1_LB = 0.0;
float Thermistor_2_LB = 0.0;
float Thermistor_3_LB = 0.0;
float Thermistor_4_LB = 0.0;

float Thermistor_1_LF = 0.0;
float Thermistor_2_LF = 0.0;
float Thermistor_3_LF = 0.0;
float Thermistor_4_LF = 0.0;

char LEVITATION_TOGGLE = 'a';
char HEMS_LEMS_PRECHARGE_DISCHARGE_TOGGLE = 'b';

sem_t faultSem;

void *mainThread(void *arg0)
{
    ConfigureDisplay();
    ConfigureTimer();
    ConfigureUART();
    ConfigureCAN();

    pthread_t           threadCAN;
    pthread_t           threadFAULT;
    pthread_attr_t      attrs;
    struct sched_param  priParam;
    int                 retc;
    int                 detachState;
    int32_t             faultSemstatus; 

    faultSemstatus = sem_init(&faultSem, 0, 0);
    if (faultSemstatus != 0)
    {
        Display_printf(display, 0, 0, "Error creating faultSem \n");
        while(1);
    }

    pthread_attr_init(&attrs);
    priParam.sched_priority = 2;
    pthread_attr_setschedparam(&attrs, &priParam);

    detachState = PTHREAD_CREATE_DETACHED;
    retc = pthread_attr_setdetachstate(&attrs, detachState);
    retc |= pthread_attr_setstacksize(&attrs, THREADSTACKSIZE);
    retc |= pthread_create(&threadCAN, &attrs, CANThread, NULL);
    if (retc != 0)
    {
        Display_printf(display, 0, 0, "CANThread failed");
        while (1);
    }

    pthread_attr_init(&attrs);
    priParam.sched_priority = 1;
    pthread_attr_setschedparam(&attrs, &priParam);

    detachState = PTHREAD_CREATE_DETACHED;
    retc = pthread_attr_setdetachstate(&attrs, detachState);
    retc |= pthread_attr_setstacksize(&attrs, THREADSTACKSIZE);
    retc |= pthread_create(&threadFAULT, &attrs, faultThread, NULL);
    if (retc != 0)
    {
        Display_printf(display, 0, 0, "faultThread failed");
        while (1);
    }

    while(1)
    {
        usleep(10000);
    }
}

void *CANThread(void *arg0)
{
    while(1)
    {
        int32_t = canStatus = CAN_read(can, &readCAN, sizeof(readCAN));
        
        if (canStatus == (-EAGAIN))
        {
            Display_printf(display, 0, 0, "No CAN message received \n");
        }

        else if (canStatus != (-EAGAIN))
        {
            Display_printf(display, 0, 0, "CAN id received 0x%8x \n", readCAN.id);

            for(uint8_t i = 0; i < readCAN.dlc; i++)
            {
                msg = (msg << 8) + readCAN.data[readCAN.dlc - 1 - i];
            }

            switch(readCAN.id)
            {
                case MCU_Control_id:
                    Levitate_toggle = (bool) parseData(0, 1, 1, 2)
                    HEMS_LEMS_Precharge_Discharge = (bool) parseData(0, 1, 1, 6);
                    
                    if (LCU_state == ACTIVE_STATE && HEMS_LEMS_Precharge_Discharge == 1)
                    {
                        LCU_state = CHARGING_STATE;
                        Display_printf(display, 0, 0, "LCU State %d: \n", LCU_state);
                        UART_write(uartRF, &HEMS_LEMS_PRECHARGE_DISCHARGE_TOGGLE, sizeof(HEMS_LEMS_PRECHARGE_DISCHARGE_TOGGLE));
                        UART_write(uartRB, &HEMS_LEMS_PRECHARGE_DISCHARGE_TOGGLE, sizeof(HEMS_LEMS_PRECHARGE_DISCHARGE_TOGGLE));
                        UART_write(uartLB, &HEMS_LEMS_PRECHARGE_DISCHARGE_TOGGLE, sizeof(HEMS_LEMS_PRECHARGE_DISCHARGE_TOGGLE));
                        UART_write(uartLF, &HEMS_LEMS_PRECHARGE_DISCHARGE_TOGGLE, sizeof(HEMS_LEMS_PRECHARGE_DISCHARGE_TOGGLE));
                    }

                    else if (LCU_state == CHARGING_STATE && Levitate_toggle == 1)
                    {
                        LCU_state = ON_STATE;
                        Display_printf(display, 0, 0, "LCU State %d: \n", LCU_state);
                        UART_write(uartRF, &LEVITATION_TOGGLE, sizeof(LEVITATION_TOGGLE));
                        UART_write(uartRB, &LEVITATION_TOGGLE, sizeof(LEVITATION_TOGGLE));
                        UART_write(uartLB, &LEVITATION_TOGGLE, sizeof(LEVITATION_TOGGLE));
                        UART_write(uartLF, &LEVITATION_TOGGLE, sizeof(LEVITATION_TOGGLE));
                    }

                    break;
                
                case LCU_Sensors_RF_id:
                    Lateral_airgap_RF = parseData(AIRGAP_OFFEST, AIRGAP_FACTOR, 10, 0);
                    Vertical_airgap_RF = parseData(AIRGAP_OFFEST, AIRGAP_FACTOR, 10, 9);
                    HEMS_current_RF = parseData(CURRENT_OFFSET, CURRENT_FACTOR, 10, 19);
                    LEMS_current_RF = parseData(CURRENT_OFFSET, CURRENT_FACTOR, 10, 29);
                    HEMS_voltage_RF = parseData(VOLTAGE_OFFSET, VOLTAGE_FACTOR, 10, 39);
                    LEMS_voltage_RF = parseData(VOLTAGE_OFFSET, VOLTAGE_FACTOR, 10, 49);

                    break;

                case LCU_Sensors_RB_id:
                    Lateral_airgap_RB = parseData(AIRGAP_OFFEST, AIRGAP_FACTOR, 10, 0);
                    Vertical_airgap_RB = parseData(AIRGAP_OFFEST, AIRGAP_FACTOR, 10, 9);
                    HEMS_current_RB = parseData(CURRENT_OFFSET, CURRENT_FACTOR, 10, 19);
                    LEMS_current_RB = parseData(CURRENT_OFFSET, CURRENT_FACTOR, 10, 29);
                    HEMS_voltage_RB = parseData(VOLTAGE_OFFSET, VOLTAGE_FACTOR, 10, 39);
                    LEMS_voltage_RB = parseData(VOLTAGE_OFFSET, VOLTAGE_FACTOR, 10, 49);

                    break;

                case LCU_Sensors_LB_id:
                    Lateral_airgap_LB = parseData(AIRGAP_OFFEST, AIRGAP_FACTOR, 10, 0);
                    Vertical_airgap_LB = parseData(AIRGAP_OFFEST, AIRGAP_FACTOR, 10, 9);
                    HEMS_current_LB = parseData(CURRENT_OFFSET, CURRENT_FACTOR, 10, 19);
                    LEMS_current_LB = parseData(CURRENT_OFFSET, CURRENT_FACTOR, 10, 29);    
                    HEMS_voltage_LB = parseData(VOLTAGE_OFFSET, VOLTAGE_FACTOR, 10, 39);
                    LEMS_voltage_LB = parseData(VOLTAGE_OFFSET, VOLTAGE_FACTOR, 10, 49);

                    break;

                case LCU_Sensors_LF_id:
                    Lateral_airgap_LF = parseData(AIRGAP_OFFEST, AIRGAP_FACTOR, 10, 0);
                    Vertical_airgap_LF = parseData(AIRGAP_OFFEST, AIRGAP_FACTOR, 10, 9);
                    HEMS_current_LF = parseData(CURRENT_OFFSET, CURRENT_FACTOR, 10, 19);
                    LEMS_current_LF = parseData(CURRENT_OFFSET, CURRENT_FACTOR, 10, 29);
                    HEMS_voltage_LF = parseData(VOLTAGE_OFFSET, VOLTAGE_FACTOR, 10, 39);
                    LEMS_voltage_LF = parseData(VOLTAGE_OFFSET, VOLTAGE_FACTOR, 10, 49);

                    break;
                
                case LCU_Temperature_RF_id:
                    Thermistor_1_RF = parseData(TEMPERATURE_OFFEST, TEMPERATURE_FACTOR, 8, 0);
                    Thermistor_2_RF = parseData(TEMPERATURE_OFFEST, TEMPERATURE_FACTOR, 8, 7);
                    Thermistor_3_RF = parseData(TEMPERATURE_OFFEST, TEMPERATURE_FACTOR, 8, 15);
                    Thermistor_4_RF = parseData(TEMPERATURE_OFFEST, TEMPERATURE_FACTOR, 8, 23);
                    
                    break;

                case LCU_Temperature_RB_id:
                    Thermistor_1_RB = parseData(TEMPERATURE_OFFEST, TEMPERATURE_FACTOR, 8, 0);
                    Thermistor_2_RB = parseData(TEMPERATURE_OFFEST, TEMPERATURE_FACTOR, 8, 7);
                    Thermistor_3_RB = parseData(TEMPERATURE_OFFEST, TEMPERATURE_FACTOR, 8, 15);
                    Thermistor_4_RB = parseData(TEMPERATURE_OFFEST, TEMPERATURE_FACTOR, 8, 23);

                    break;

                case LCU_Temperature_LB_id:
                    Thermistor_1_LB = parseData(TEMPERATURE_OFFEST, TEMPERATURE_FACTOR, 8, 0);
                    Thermistor_2_LB = parseData(TEMPERATURE_OFFEST, TEMPERATURE_FACTOR, 8, 7);
                    Thermistor_3_LB = parseData(TEMPERATURE_OFFEST, TEMPERATURE_FACTOR, 8, 15);
                    Thermistor_4_LB = parseData(TEMPERATURE_OFFEST, TEMPERATURE_FACTOR, 8, 23);

                    break;

                case LCU_Temperature_LF_id:
                    Thermistor_1_LF = parseData(TEMPERATURE_OFFEST, TEMPERATURE_FACTOR, 8, 0);
                    Thermistor_2_LF = parseData(TEMPERATURE_OFFEST, TEMPERATURE_FACTOR, 8, 7);
                    Thermistor_3_LF = parseData(TEMPERATURE_OFFEST, TEMPERATURE_FACTOR, 8, 15);
                    Thermistor_4_LF = parseData(TEMPERATURE_OFFEST, TEMPERATURE_FACTOR, 8, 23);

                    break;

                case LCU_Parameters_id:
                    LCU_Parameters_Acknowledgement.id = LCU_Parameters_Acknowledgement_id;
                    LCU_Parameters_Acknowledgement.dlc = readCAN.dlc;
                    for (uint8_t i = 0; i < readCAN.dlc; i++)
                    {
                        LCU_Parameters_Acknowledgement.data[i] = readCAN.data[i];
                    }

                    Display_printf(display, 0, 0, "LCU Parameters Acknowledgment sent \n");
                    CAN_write(can, &LCU_Parameters_Acknowledgement, sizeof(LCU_Parameters_Acknowledgement));

                    break;
            }

            faultCheck();
            msg = 0;
        }
    }
}

void *faultThread(void *arg0)
{
    sem_wait faultSem;
    while(1)
    {   
        if(faultCheck())
        {
            LCU_state = FAULT_STATE;
            
            LCU_Fault_Message.id = LCU_Fault_Message_id;
            LCU_Fault_Message.dlc = 1; 
            LCU_Fault_Message.data[0] = faultCheck(); 

            Display_printf(display, 0, 0, "FAULT STATE \n");
            CAN_write(can, &LCU_Fault_Message, sizeof(LCU_Fault_Message));
        }
    }
}

void ConfigureDisplay(void)
{
    Display_init();

    Display_Params displayParams;
    Display_Params_init(&displayParams);
    display = Display_open(Display_Type_UART, &displayParams);
    Display_printf(display, 0, 0, "Display initialized \n");
}

void ConfigureUART(void)
{
    // PA7(1) - tx, PA6(2) - rx, J33
    UART_init();

    UART_Params uartParams;
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.readReturnMode = UART_RETURN_FULL;
    uartParams.baudRate = 115200;

    uartRF = UART_open(CONFIG_UART_RF, &uartParams);
    uartRB = UART_open(CONFIG_UART_RB, &uartParams);
    uartLB = UART_open(CONFIG_UART_LB, &uartParams);
    uartLF = UART_open(CONFIG_UART_LF, &uartParams);
    
    if (uartRF == NULL || uartRB == NULL || uartLB == NULL || uartLF == NULL)
    {
        Display_printf(display, 0, 0, "UART failed to open \n");
        while (1);
    }

    Display_printf(display, 0, 0, "UART started successfully \n");
}

void ConfigureCAN(void)
{
    // PA1(1, 7) - tx, PA0(2, 8) - rx, J28
    CAN_init();

    CAN_Params_init(&canParams);
    canParams.filterID = 0x00000000; 
    canParams.filterMask = 0x00000F00;
    canParams.mode = CAN_MODE_NONBLOCKING;

    can = CAN_open(CONFIG_CAN, &canParams);

    if (can == NULL)
    {
        Display_printf(display, 0, 0, "CAN configuration failed");
        while(1);
    }

    Display_printf(display, 0, 0, "CAN started successsfully \n");
}

float parseData(float offset, float factor, int len, int s_bit)
{
    uint64_t buffer = 0;
    float parsed_msg = 0;

    buffer = msg << (64 - (s_bit + len));
    buffer = buffer >> (64 - len);
    parsed_msg = ((float)buffer * factor) + offset;

    return parsed_msg;
}

uint8_t faultCheck(void)
{
    if(HEMS_voltage_RF > VOLTAGE_MAX || HEMS_voltage_RB > VOLTAGE_MAX ||
       HEMS_voltage_LB > VOLTAGE_MAX || HEMS_voltage_LF > VOLTAGE_MAX)    
    {
        return 0b00001000;
    }

    else if(LEMS_voltage_RF > VOLTAGE_MAX || LEMS_voltage_RB > VOLTAGE_MAX ||
            LEMS_voltage_LB > VOLTAGE_MAX || LEMS_voltage_LF > VOLTAGE_MAX)
    {
        return 0b00000100;
    }

    else if(HEMS_current_RF > CURRENT_MAX || HEMS_current_RB > CURRENT_MAX ||
            HEMS_current_LB > CURRENT_MAX || HEMS_current_LF > CURRENT_MAX)
    {
        return 0b00100000;
    }

    else if(LEMS_current_RF > CURRENT_MAX || LEMS_current_RB > CURRENT_MAX ||
            LEMS_current_LB > CURRENT_MAX || LEMS_current_LF > CURRENT_MAX)
    {
        return 0b00010000;
    }

    else if(Lateral_airgap_RF < AIRGAP_MIN || Lateral_airgap_RB < AIRGAP_MIN ||
            Lateral_airgap_LB < AIRGAP_MIN || Lateral_airgap_LF < AIRGAP_MIN
            Lateral_airgap_RF > AIRGAP_MAX || Lateral_airgap_RB > AIRGAP_MAX ||
            Lateral_airgap_LB > AIRGAP_MAX || Lateral_airgap_LF > AIRGAP_MAX)
    {
        return 0b10000000;
    }

    else if(Vertical_airgap_RF < AIRGAP_MIN || Vertical_airgap_RB < AIRGAP_MIN ||
            Vertical_airgap_LB < AIRGAP_MIN || Vertical_airgap_LF < AIRGAP_MIN ||
            Vertical_airgap_RF > AIRGAP_MAX || Vertical_airgap_RB > AIRGAP_MAX ||
            Vertical_airgap_LB > AIRGAP_MAX || Vertical_airgap_LF > AIRGAP_MAX)
    {
        return 0b01000000;
    }

    else if(Thermistor_1_RF < TEMPERATURE_MIN || Thermistor_2_RF < TEMPERATURE_MIN ||
            Thermistor_1_RB < TEMPERATURE_MIN || Thermistor_2_RB < TEMPERATURE_MIN ||
            Thermistor_1_LB < TEMPERATURE_MIN || Thermistor_2_LB < TEMPERATURE_MIN ||
            Thermistor_1_LF < TEMPERATURE_MIN || Thermistor_2_LF < TEMPERATURE_MIN ||
            Thermistor_1_RF > TEMPERATURE_MAX || Thermistor_2_RF > TEMPERATURE_MAX ||
            Thermistor_1_RB > TEMPERATURE_MAX || Thermistor_2_RB > TEMPERATURE_MAX ||
            Thermistor_1_LF > TEMPERATURE_MAX || Thermistor_2_LF > TEMPERATURE_MAX ||
            Thermistor_1_LB > TEMPERATURE_MAX || Thermistor_2_LB > TEMPERATURE_MAX)
    {
        return 0b00000010;
    }

    else if(Thermistor_3_RF < TEMPERATURE_MIN || Thermistor_4_RF < TEMPERATURE_MIN ||
            Thermistor_3_RB < TEMPERATURE_MIN || Thermistor_4_RB < TEMPERATURE_MIN ||
            Thermistor_3_LB < TEMPERATURE_MIN || Thermistor_4_LB < TEMPERATURE_MIN ||
            Thermistor_3_LF < TEMPERATURE_MIN || Thermistor_4_LF < TEMPERATURE_MIN ||
            Thermistor_3_RF > TEMPERATURE_MAX || Thermistor_4_RF > TEMPERATURE_MAX ||
            Thermistor_3_RB > TEMPERATURE_MAX || Thermistor_4_RB > TEMPERATURE_MAX ||
            Thermistor_3_LF > TEMPERATURE_MAX || Thermistor_4_LF > TEMPERATURE_MAX ||
            Thermistor_3_LB > TEMPERATURE_MAX || Thermistor_4_LB > TEMPERATURE_MAX)
    {
        return 0b00000001;
    }

    return 0;
}