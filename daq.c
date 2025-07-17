#include <stdint.h>
#include <stddef.h>
#include <math.h>

#include <semaphore.h>
#include <unistd.h>
#include <pthread.h>

#include <ti/display/Display.h>
#include <ti/drivers/Timer.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/UART.h>
#include <ti/display/DisplayUart.h>
#include <ti/drivers/CAN.h>
#include <ti/drivers/ADCBuf.h>

#include "ti_drivers_config.h"

Display_Handle display;
static CAN_Handle can;
ADCBuf_Handle adcBuf;

void ConfigureDisplay(void);
void ConfigureTimer(void);
void ConfigureGPIO(void);
void ConfigureDIP(void);
void ConfigureCAN(void);
void ConfigureADC(void);

void timerCallback(Timer_Handle myHandle, int_fast16_t status);
void adcBufCallback(ADCBuf_Handle handle, ADCBuf_Conversion *conversion, void *completedADCBuffer, uint32_t completedChannel, int_fast16_t status);

void *txThread(void *arg0);

void packageData(uint8_t* data, uint64_t msg, uint8_t dlc);

float thermistor(int voltage);
float thermocouple(int voltage);
float pressure(int voltage);
float airgap(int voltage);
int proximity(int voltage);

uint64_t daq_sensors_msg(void);
uint64_t hems_temperature_msg(void);
uint64_t lems_temperature_msg(void);
uint64_t lim_temperature_msg(void);
uint64_t pressure_msg_1(void);
uint64_t pressure_msg_2(void);
uint64_t pcm_temperature_msg_1(void);
uint64_t pcm_temperature_msg_2(void);

CAN_Frame daq_sensors_1_lb(void);
CAN_Frame daq_sensors_1_rf(void);
CAN_Frame daq_sensors_2_lf(void);
CAN_Frame daq_sensors_2_rb(void);
CAN_Frame hems_temperature_rf(void);
CAN_Frame hems_temperature_rb(void);
CAN_Frame hems_temperature_lb(void);
CAN_Frame hems_temperature_lf(void);
CAN_Frame lems_temperature_rf(void);
CAN_Frame lems_temperature_rb(void);
CAN_Frame lems_temperature_lb(void);
CAN_Frame lim_temperature_lf(void);
CAN_Frame lim_temperature_rb(void);
CAN_Frame thermal_pressure_1(void);
CAN_Frame thermal_pressure_2(void);
CAN_Frame pcm_temperature_1(void);
CAN_Frame pcm_temperature_2(void);

CAN_Frame DAQ_Sensors_1_LB = {0};
CAN_Frame DAQ_Sensors_1_RF = {0};

CAN_Frame DAQ_Sensors_2_LF = {0};
CAN_Frame DAQ_Sensors_2_RB = {0};

CAN_Frame HEMS_Temperature_RF = {0};
CAN_Frame HEMS_Temperature_RB = {0};
CAN_Frame HEMS_Temperature_LB = {0};
CAN_Frame HEMS_Temperature_LF = {0};

CAN_Frame LEMS_Temperature_RF = {0};
CAN_Frame LEMS_Temperature_RB = {0};
CAN_Frame LEMS_Temperature_LB = {0};
CAN_Frame LEMS_Temperature_LF = {0};

CAN_Frame LIM_Temperature_LF = {0};
CAN_Frame LIM_Temperature_RB = {0};

CAN_Frame Thermal_Pressure_1 = {0};
CAN_Frame Thermal_Pressure_2 = {0};

CAN_Frame PCM_Temperature_1 = {0};
CAN_Frame PCM_Temperature_2 = {0};

#define DAQ_Sensors_1_LB_id 0x04003FFF
#define DAQ_Sensors_1_RF_id 0x04013FFF

#define DAQ_Sensors_2_LF_id 0x04023FFF
#define DAQ_Sensors_2_RB_id 0x04033FFF

#define HEMS_Temperature_RF_id 0x008077FF
#define HEMS_Temperature_RB_id 0x00807BFF
#define HEMS_Temperature_LB_id 0x00807DFF
#define HEMS_Temperature_LF_id 0x00807EFF 

#define LEMS_Temperature_RF_id 0x008177FF
#define LEMS_Temperature_RB_id 0x00817BFF
#define LEMS_Temperature_LB_id 0x00817DFF
#define LEMS_Temperature_LF_id 0x00817EFF 

#define LIM_Temperature_LF_id 0x00821EFF
#define LIM_Temperature_RB_id 0x00821BFF

#define Thermal_Pressure_1_id 0x02025FFF
#define Thermal_Pressure_2_id 0x02035FFF

#define PCM_Temperature_1_id 0x02005FFF
#define PCM_Temperature_2_id 0x02015FFF

#define TEMPERATURE_OFFEST (0)
#define TEMPERATURE_FACTOR (1000)

#define PRESSURE_OFFEST (0)
#define PRESSURE_FACTOR (1000)

#define AIRGAP_OFFEST (0)
#define AIRGAP_FACTOR (1000)

#define CONFIG_GPIO_ON (1)
#define CONFIG_GPIO_OFF (0)

#define THREADSTACKSIZE    1024

#define SAMPLESIZE (4)
uint16_t sampleBuffer1[SAMPLESIZE];
uint16_t sampleBuffer2[SAMPLESIZE];
uint16_t sampleBuffer3[SAMPLESIZE];
uint16_t sampleBuffer4[SAMPLESIZE];

uint16_t outputBuffer[12][SAMPLESIZE];

uint16_t thermistor_outputBuffer[8 * SAMPLESIZE];
uint16_t thermocouple_pressure_airgap_outputBuffer[4 * SAMPLESIZE];
uint32_t thermistor_buffersCompletedCounter = 0;
uint32_t thermocouple_pressure_airgap_buffersCompletedCounter = 0;

sem_t txSem;
sem_t adcbufSem;

bool dip_switches[5];

void *mainThread(void *arg0)
{
    ConfigureDisplay();
    ConfigureTimer();
    ConfigureGPIO();
    ConfigureDIP();
    ConfigureUART();
    ConfigureCAN();

    ADCBuf_init();
    ConfigureADC0();
    ConfigureADC1();

    pthread_t           thread0;
    pthread_attr_t      attrs;
    struct sched_param  priParam;
    int                 retc;
    int                 detachState;
    int32_t             txSemstatus;

    txSemstatus = sem_init(&txSem, 0, 0);
    if (txSemstatus != 0)
    {
        Display_printf(display, 0, 0, "Error creating txSem \n");
        while(1);
    }

    pthread_attr_init(&attrs);
    priParam.sched_priority = 1;
    pthread_attr_setschedparam(&attrs, &priParam);

    detachState = PTHREAD_CREATE_DETACHED;
    retc = pthread_attr_setdetachstate(&attrs, detachState);
    retc |= pthread_attr_setstacksize(&attrs, THREADSTACKSIZE);
    retc |= pthread_create(&thread0, &attrs, txThread, NULL);
    if (retc != 0)
    {
        Display_printf(display, 0, 0, "txThread failed");
        while (1);
    }

    while(1) 
    {
        sem_wait(&thermistor_adcbufSem);
        sem_wait(&thermocouple_pressure_airgap_adcbufSem);

        sem_post(&txSem);
        
        Display_printf(display, 0, 0, "Thermistor Buffer %u finished \n", (unsigned int)thermistor_buffersCompletedCounter++);
        Display_printf(display, 0, 0, "Thermocouple_Pressure_Airgap Buffer %u finished \n", (unsigned int)thermocouple_pressure_airgap_buffersCompletedCounter++);

        uint_fast16_t i;
        uint_fast16_t j;
        for (i = 0; i < 8 * SAMPLESIZE; i++) 
        {
            Display_printf(display, 0, 0, "%u, ", thermistor_outputBuffer[i]);
        }
        Display_printf(display, 0, 0, "\n");

        for (j = 0; j < 4 * SAMPLESIZE; j++)
        {
            Display_printf(display, 0, 0, "%u, ", thermocouple_pressure_airgap_outputBuffer[j]);
        }
        Display_printf(display, 0, 0, "\n");
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

void ConfigureTimer(void)
{
    Timer_init();

    Timer_Handle Timer;
    Timer_Params timer_params;

    Timer_Params_init(&timer_params);
    timer_params.period = 12500; //for transmitting all messages with 50ms time period
    timer_params.periodUnits = Timer_PERIOD_US;
    timer_params.timerMode = Timer_CONTINUOUS_CALLBACK;
    timer_params.timerCallback = timerCallback;

    Timer = Timer_open(CAN_TIMER, &timer_params); // timer2

    if (Timer == NULL)
    {
        Display_printf(display, 0, 0, "Timer initialization error \n");
        while(1);
    }

    if (Timer_start(Timer) == Timer_STATUS_ERROR)
    {
        Display_printf(display, 0, 0, "Timer start error \n");
        while(1);
    }

    Display_printf(display, 0, 0, "Timer started successfully \n");
}

void ConfigureGPIO()
{
    GPIO_init();

    GPIO_setConfig(HEMS_SEL, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(LEMS_SEL, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(LIM_SEL_1, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(LIM_SEL_2, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(PSel_1_2, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(PSel_3_4, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(PSel_5_6, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(PSel_7_8, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(Press_Sel_1, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(Press_Sel_2, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(Press_Sel_3, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(Press_Sel_4, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(Proximity_Up, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);
    GPIO_setConfig(Proximity_Down, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);


    GPIO_write(HEMS_SEL, CONFIG_GPIO_OFF);
    GPIO_write(LEMS_SEL, CONFIG_GPIO_OFF);
    GPIO_write(LIM_SEL_1, CONFIG_GPIO_OFF);
    GPIO_write(LIM_SEL_2, CONFIG_GPIO_OFF);
    GPIO_write(PSel_1_2, CONFIG_GPIO_OFF);
    GPIO_write(PSel_3_4, CONFIG_GPIO_OFF);
    GPIO_write(PSel_5_6, CONFIG_GPIO_OFF);
    GPIO_write(PSel_7_8, CONFIG_GPIO_OFF);
    GPIO_write(Press_Sel_1, CONFIG_GPIO_OFF);
    GPIO_write(Press_Sel_2, CONFIG_GPIO_OFF);
    GPIO_write(Press_Sel_3, CONFIG_GPIO_OFF);
    GPIO_write(Press_Sel_4, CONFIG_GPIO_OFF);
}

void ConfigureDIP(void)
{
    /*
    * DP1-4, DP5 : PP5-2, PQ4
    */
    GPIO_setConfig(DIP_1, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);
    GPIO_setConfig(DIP_2, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);
    GPIO_setConfig(DIP_3, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);
    GPIO_setConfig(DIP_4, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);
    GPIO_setConfig(DIP_5, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

    dip_switches[0] = GPIO_read(DIP_1);
    dip_switches[1] = GPIO_read(DIP_2);
    dip_switches[2] = GPIO_read(DIP_3);
    dip_switches[3] = GPIO_read(DIP_4);
    dip_switches[4] = GPIO_read(DIP_5);
}

void ConfigureCAN(void)
{
    // PA1(1, 7) - tx, PA0(2, 8) - rx, J28
    CAN_init();
    CAN_Params canParams;
    CAN_Params_init(&canParams);

    if(dip_switches[0] == 1)
    {
        canParams.filterID = 0x00000070;
    }

    else if(dip_switches[1] == 1)
    {
        canParams.filterID = 0x000000B0; 
    }

    else if(dip_switches[2] == 1)
    {
        canParams.filterID = 0x000000D0; 
    }

    else if(dip_switches[3] == 1)
    {
        canParams.filterID = 0x000000E0; 
    }

    else if(dip_switches[4] == 1)
    {
        canParams.filterID = 0x00000000; 
    }

    canParams.filterMask = 0x000000F0;
    canParams.mode = CAN_MODE_NONBLOCKING;

    can = CAN_open(CONFIG_CAN, &canParams);

    if (can == NULL)
    {
        Display_printf(display, 0, 0, "CAN configuration failed");
        while(1);
    }

    Display_printf(display, 0, 0, "CAN started successsfully \n");
}

void ConfigureADC(void)
{
    int32_t ADCstatus;
    ADCstatus = sem_init(&adcbufSem, 0, 0);
    
    if (ADCstatus != 0)
    {
        Display_printf(display, 0, 0, "Error creating adcbufSem \n");
        while(1);
    }

    ADCBuf_Params adcBufParams;   
    ADCBuf_Params_init(&adcBufParams);
    adcBufParams_thermistor.callbackFxn = adcBufCallback;
    adcBufParams_thermistor.recurrenceMode = ADCBuf_RECURRENCE_MODE_CONTINUOUS;
    adcBufParams_thermistor.returnMode = ADCBuf_RETURN_MODE_CALLBACK;
    adcBufParams_thermistor.samplingFrequency = 80;
    adcBuf = ADCBuf_open(ADCBUF, &adcBufParams);

    ADCBuf_Conversion continuousConversion_thermistor[8];
    ADCBuf_Conversion continuousConversion_thermocouple_pressure_airgap[4];

    continuousConversion_thermistor[0].arg = NULL;
    continuousConversion_thermistor[0].adcChannel = HEMS_TEMPERATURE_1;
    continuousConversion_thermistor[0].sampleBuffer = sampleBuffer1;
    continuousConversion_thermistor[0].sampleBufferTwo = sampleBuffer2;
    continuousConversion_thermistor[0].samplesRequestedCount = HEMS_LEMS_SAMPLESIZE;

    continuousConversion_thermistor[1].arg = NULL;
    continuousConversion_thermistor[1].adcChannel = HEMS_TEMPERATURE_2;
    continuousConversion_thermistor[1].sampleBuffer = NULL;
    continuousConversion_thermistor[1].sampleBufferTwo = NULL;
    continuousConversion_thermistor[1].samplesRequestedCount = HEMS_LEMS_SAMPLESIZE;
    
    continuousConversion_thermistor[2].arg = NULL;
    continuousConversion_thermistor[2].adcChannel = LEMS_TEMPERATURE_1;
    continuousConversion_thermistor[2].sampleBuffer = NULL;
    continuousConversion_thermistor[2].sampleBufferTwo = NULL;
    continuousConversion_thermistor[2].samplesRequestedCount = HEMS_LEMS_SAMPLESIZE;

    continuousConversion_thermistor[3].arg = NULL;
    continuousConversion_thermistor[3].adcChannel = LEMS_TEMPERATURE_2;
    continuousConversion_thermistor[3].sampleBuffer = NULL;
    continuousConversion_thermistor[3].sampleBufferTwo = NULL;
    continuousConversion_thermistor[3].samplesRequestedCount = HEMS_LEMS_SAMPLESIZE;
    
    continuousConversion_thermistor[4].arg = NULL;
    continuousConversion_thermistor[4].adcChannel = LIM_TEMPERATURE_1;
    continuousConversion_thermistor[4].sampleBuffer = NULL;
    continuousConversion_thermistor[4].sampleBufferTwo = NULL;
    continuousConversion_thermistor[4].samplesRequestedCount = LIM_SAMPLESIZE;

    continuousConversion_thermistor[5].arg = NULL;
    continuousConversion_thermistor[5].adcChannel = LIM_TEMPERATURE_2;
    continuousConversion_thermistor[5].sampleBuffer = NULL;
    continuousConversion_thermistor[5].sampleBufferTwo = NULL;
    continuousConversion_thermistor[5].samplesRequestedCount = LIM_SAMPLESIZE;
    
    continuousConversion_thermistor[6].arg = NULL;
    continuousConversion_thermistor[6].adcChannel = LIM_TEMPERATURE_3;
    continuousConversion_thermistor[6].sampleBuffer = NULL;
    continuousConversion_thermistor[6].sampleBufferTwo = NULL;
    continuousConversion_thermistor[6].samplesRequestedCount = LIM_SAMPLESIZE;
    
    continuousConversion_thermistor[7].arg = NULL;
    continuousConversion_thermistor[7].adcChannel = LIM_TEMPERATURE_4;
    continuousConversion_thermistor[7].sampleBuffer = NULL;
    continuousConversion_thermistor[7].sampleBufferTwo = NULL;
    continuousConversion_thermistor[7].samplesRequestedCount = LIM_SAMPLESIZE;
  

    continuousConversion_thermocouple_pressure_airgap[0].arg = NULL;
    continuousConversion_thermocouple_pressure_airgap[0].adcChannel = PRESSURE;
    continuousConversion_thermocouple_pressure_airgap[0].sampleBuffer = sampleBuffer3;
    continuousConversion_thermocouple_pressure_airgap[0].sampleBufferTwo = sampleBuffer4;
    continuousConversion_thermocouple_pressure_airgap[0].samplesRequestedCount = PRESSURE_SAMPLESIZE;

    continuousConversion_thermocouple_pressure_airgap[1].arg = NULL;
    continuousConversion_thermocouple_pressure_airgap[1].adcChannel = PRESSURE_AIRGAP;
    continuousConversion_thermocouple_pressure_airgap[1].sampleBuffer = NULL;
    continuousConversion_thermocouple_pressure_airgap[1].sampleBufferTwo = NULL;
    continuousConversion_thermocouple_pressure_airgap[1].samplesRequestedCount = PRESSURE_AIRGAP_SAMPLESIZE;

    continuousConversion_thermocouple_pressure_airgap[2].arg = NULL;
    continuousConversion_thermocouple_pressure_airgap[2].adcChannel = PCM_TEMPERATURE_1;
    continuousConversion_thermocouple_pressure_airgap[2].sampleBuffer = NULL;
    continuousConversion_thermocouple_pressure_airgap[2].sampleBufferTwo = NULL;
    continuousConversion_thermocouple_pressure_airgap[2].samplesRequestedCount = PCM_TEMPERATURE; 

    continuousConversion_thermocouple_pressure_airgap[3].arg = NULL;
    continuousConversion_thermocouple_pressure_airgap[3].adcChannel = PCM_TEMPERATURE_2;
    continuousConversion_thermocouple_pressure_airgap[3].sampleBuffer = NULL;
    continuousConversion_thermocouple_pressure_airgap[3].sampleBufferTwo = NULL;
    continuousConversion_thermocouple_pressure_airgap[3].samplesRequestedCount = PCM_TEMPERATURE;

    if (!adcBuf)
    {
        Display_printf(display, 0, 0, "adcBuf open failed \n");
        while(1);
    }

    if (ADCBuf_convert(adcBuf, continuousConversion_thermistor, 8) != ADCBuf_STATUS_SUCCESS)
    {
        Display_printf(display, 0, 0, "Thermistor conversion failed \n");
        while(1);
    }

    if (ADCBuf_convert(adcBuf, continuousConversion_thermocouple_pressure_airgap, 4) != ADCBuf_STATUS_SUCCESS)
    {
        Display_printf(display, 0, 0, "Thermocouple, Pressure, Airgap conversion failed \n");
        while(1);
    }

}

uint64_t t = 0;
void timerCallback(Timer_Handle myHandle, int_fast16_t status)
{
    t++;

    GPIO_toggle(Press_Sel_1);
    GPIO_toggle(Press_Sel_3);

    if(t % 2 == 0)
    {
        GPIO_toggle(HEMS_SEL);
        GPIO_toggle(LEMS_SEL);
        GPIO_toggle(LIM_SEL_1);
        GPIO_toggle(LIM_SEL_2);

        GPIO_toggle(Press_Sel_2);
        GPIO_toggle(Press_Sel_4);
    }
}

void adcBufCallback(ADCBuf_Handle handle, ADCBuf_Conversion *conversion, void *completedADCBuffer, uint32_t completedChannel, int_fast16_t status)
{
    uint_fast16_t a;
    uint16_t *completedBuffer = (uint16_t *) completedADCBuffer;

    for (a = 0; a < SAMPLESIZE; a++)
    {
        outputBuffer[completedChannel][a] = completedBuffer[a];
    }
    
    if(completedChannel == HEMS_TEMPERATURE_1)
    {
        uint_fast16_t b, c; 
        for (b = 0; b < SAMPLESIZE; b++)
        {
            thermistor_outputBuffer[b] = outputBuffer[HEMS_TEMPERATURE_1][b];
            thermistor_outputBuffer[b + SAMPLE_SIZE] = outputBuffer[HEMS_TEMPERATURE_2][b];
            thermistor_outputBuffer[b + 2 * SAMPLE_SIZE] = outputBuffer[LEMS_TEMPERATURE_1][b];
            thermistor_outputBuffer[b + 3 * SAMPLE_SIZE] = outputBuffer[LEMS_TEMPERATURE_2][b];
            thermistor_outputBuffer[b + 4 * SAMPLE_SIZE] = outputBuffer[LIM_TEMPERATURE_1][b];
            thermistor_outputBuffer[b + 5 * SAMPLE_SIZE] = outputBuffer[LIM_TEMPERATURE_2][b];
            thermistor_outputBuffer[b + 6 * SAMPLE_SIZE] = outputBuffer[LIM_TEMPERATURE_3][b];
            thermistor_outputBuffer[b + 7 * SAMPLE_SIZE] = outputBuffer[LIM_TEMPERATURE_4][b];
        }
        
        for (c = 0; c < SAMPLE_SIZE; c++)
        {
            thermocouple_pressure_airgap_outputBuffer[c] = outputBuffer[PRESSURE][c];
            thermocouple_pressure_airgap_outputBuffer[c + SAMPLE_SIZE] = outputBuffer[PRESSURE_AIRGAP][c];
            thermocouple_pressure_airgap_outputBuffer[c + 2 * SAMPLE_SIZE] = outputBuffer[PCM_TEMPERATURE_1][c];
            thermocouple_pressure_airgap_outputBuffer[c + 3 * SAMPLE_SIZE] = outputBuffer[PCM_TEMPERATURE_2][c];
        }
        sem_post(&adcbufSem);
    }
}

void *txThread(void *arg0)
{
    while(1)
    {
        sem_wait(&txSem);

        if(dip_switches[0] == 1)
        {
            CAN_write(can, &DAQ_Sensors_1_RF, sizeof(DAQ_Sensors_1_RF));
            CAN_write(can, &HEMS_Temperature_RF, sizeof(HEMS_Temperature_RF));
            CAN_write(can, &LEMS_Temperature_RF, sizeof(LEMS_Temperature_RF));
            Display_printf(display, 0, 0, "DAQ_Sensors_1_RF 0x%8x sent \n", DAQ_Sensors_1_RF.id);
            Display_printf(display, 0, 0, "HEMS_Temperature_RF 0x%8x sent \n", HEMS_Temperature_RF.id);
            Display_printf(display, 0, 0, "LEMS_Temperature_RF 0x%8x sent \n", LEMS_Temperature_RF.id);
        }

        else if(dip_switches[1] == 1)
        {
            CAN_write(can, &DAQ_Sensors_2_RB, sizeof(DAQ_Sensors_2_RB));
            CAN_write(can, &HEMS_Temperature_RB, sizeof(HEMS_Temperature_RB));
            CAN_write(can, &LEMS_Temperature_RB, sizeof(LEMS_Temperature_RB));
            CAN_write(can, &LIM_Temperature_RB, sizeof(LIM_Temperature_RB));
            Display_printf(display, 0, 0, "DAQ_Sensors_2_RB 0x%8x sent \n", DAQ_Sensors_2_RB.id);
            Display_printf(display, 0, 0, "HEMS_Temperature_RB 0x%8x sent \n", HEMS_Temperature_RB.id);
            Display_printf(display, 0, 0, "LEMS_Temperature_RB 0x%8x sent \n", LEMS_Temperature_RB.id);
            Display_printf(display, 0, 0, "LIM_Temperature_RB 0x%8x sent \n", LIM_Temperature_RB.id);
        }

        else if(dip_switches[2] == 1)
        {
            CAN_write(can, &DAQ_Sensors_1_LB, sizeof(DAQ_Sensors_1_LB));
            CAN_write(can, &HEMS_Temperature_LB, sizeof(HEMS_Temperature_LB));
            CAN_write(can, &LEMS_Temperature_LB, sizeof(LEMS_Temperature_LB));
            Display_printf(display, 0, 0, "DAQ_Sensors_1_LB 0x%8x sent \n", DAQ_Sensors_1_LB.id);
            Display_printf(display, 0, 0, "HEMS_Temperature_LB 0x%8x sent \n", HEMS_Temperature_LB.id);
            Display_printf(display, 0, 0, "LEMS_Temperature_LB 0x%8x sent \n", LEMS_Temperature_LB.id);
        }

        else if(dip_switches[3] == 1)
        {
            CAN_write(can, &DAQ_Sensors_2_LF, sizeof(DAQ_Sensors_2_LF));
            CAN_write(can, &HEMS_Temperature_LF, sizeof(HEMS_Temperature_LF));
            CAN_write(can, &LEMS_Temperature_LF, sizeof(LEMS_Temperature_LF));
            CAN_write(can, &LIM_Temperature_LF, sizeof(LIM_Temperature_LF));
            Display_printf(display, 0, 0, "DAQ_Sensors_2_LF 0x%8x sent \n", DAQ_Sensors_2_LF.id);
            Display_printf(display, 0, 0, "HEMS_Temperature_LF 0x%8x sent \n", HEMS_Temperature_LF.id);
            Display_printf(display, 0, 0, "LEMS_Temperature_LF 0x%8x sent \n", LEMS_Temperature_LF.id);
            Display_printf(display, 0, 0, "LIM_Temperature_LF 0x%8x sent \n", LIM_Temperature_LF.id);
        }

        else if(dip_switches[4] == 1)
        {
            CAN_write(can, &Thermal_Pressure_1, sizeof(Thermal_Pressure_1));
            CAN_write(can, &Thermal_Pressure_2, sizeof(Thermal_Pressure_2));
            CAN_write(can, &PCM_Temperature_1, sizeof(PCM_Temperature_1));
            CAN_write(can, &PCM_Temperature_2, sizeof(PCM_Temperature_2));
            Display_printf(display, 0, 0, "Thermal_Pressure_1 0x%8x sent \n", Thermal_Pressure_1.id);
            Display_printf(display, 0, 0, "Thermal_Pressure_2 0x%8x sent \n", Thermal_Pressure_2.id);
            Display_printf(display, 0, 0, "PCM_Temperature_1 0x%8x sent \n", PCM_Temperature_1.id);
            Display_printf(display, 0, 0, "PCM_Temperature_2 0x%8x sent \n", PCM_Temperature_2.id);
        }
    }
}

void packageData(uint8_t* data, uint64_t msg, uint8_t dlc)
{
    uint8_t i;
    for (i = 0; i < dlc; i++)
    {
        data[i] = (msg >> (i * 8)) & 0xFF;
    }
}

float thermistor(int voltage)
{
    /*
     * Max 16 thermistors
     * Using 8 thermistors for LB & RF, 16 thermistors for LF & RB
     * LEMS & HEMS use 4 thermistors each
     * LIM uses 8 thermistors
     * HEMS - Therm_Inp_1-4 (J6,7; pin2,4), Therm_Out_1-2 (PD7,6), Therm_Sel_1(PQ1; low gives 2,3)
     * LEMS - Therm_Inp_5-8 (J14,15; pin2,4), Therm_Out_3-4 (PD1,0), Therm_Sel_2(PQ0; low give 6,7)
     * LIM - Therm_Inp_9-16 (J20,21,26,27; pin2,4), Therm_Out_5-8 (PD2,3 PK0,1), Therm_Sel_3-4 (PQ2, PE3; low gives 10,11,14,15)
     */
    int BETA = 3977;
    int R0 = 10000;
    float T0 = 298.15;

    double V = 0.0;
    V = 3.3 * voltage / 4095;

    double resistance = (4500 / V) - 900;


    float T = (1 / T0) + (log(resistance / R0) / BETA);

    return (1/T) - 273.15; //temperature in C
}

float thermocouple(int voltage)
{
    /*
    * Max 12 thermocouples
    * Using 8 thermocouples for PCM temperature
    * TCP_Inp_1-4 (J5, 4, 5, 4; pin 1,3(P), pin 2,4(N)), TC_Sel_1-3 (PQ3, PC4,5), ThermCouple_Out_1 (PK2)
    * TCP_Inp_9-12 (J17, 16, 17, 16; pin 1,3(P), pin 2,4(N)), TC_Sel_4-5 (PC6,7), ThermCouple_Out_2 (PK3)
    */
    double V = 0.0;
    V = 3.3 * voltage / 4095;
    double T = 0.0;
    T = (V - 1.65) * 200; //temperature in C

    return T;
}

float pressure(int voltage)
{
    /*
    * Max 12 pressure sensors
    * Using 6 pressure sensors for PCM pressure
    * Press_1-6 (J2,3,8,9,12,13; pin 2), Press_Out_1-2 (PB5,4), PSel_1-2,3-4,5-6 (PN2,3,4; 1 gives current based), Press_Sel_1-4 (PN0,1, PP0,1; Press_Sel_4 = 0)
    * Using 1 for Brake or LIM reservoir pressure
    * Using 1 for LIM airgap
    * Press_7-8 (J18,19; pin 2), Press_Out_2 (PB4), PSel_7-8 (PN5; 1 gives current based), Press_Sel_3-4 (PP0,1; Press_Sel_4 = 1)
    */
    double V = 3.3 * voltage / 4095;
    double I = V * 4; // current in mA
    double P = (I - 4) / 16; // pressure in MPa
    P = P * 1000000; //pressure in Pa
    return P;
}

float airgap(int voltage)
{
    double V = 3.3 * voltage / 4095;
    double AG = 3 * V + 35; // airgap in mm
    return AG;
}

int proximity(int gpio) // Shouldn't this be uint8_t for GPIO?x
{
    /*
     * Max 4 proximity sensors
     * Using 2 proximity sensors for brakes
     * IR_Input_1-2 (J29,30; pin 2), IR_1-2 (PK6,7)
     */
    
    if (gpio == CONFIG_GPIO_ON)
    {
        return 1;
    }
    
    else if (gpio == CONFIG_GPIO_OFF)
    {
        return 0;
    }
}

uint64_t daq_sensors_msg(void)
{
    uint64_t msg;
    msg = 0;

    bool proximity_up = GPIO_read(Proximity_Up);
    bool proximity_down = GPIO_read(Proximity_Down);
    msg += proximity_up << 13;
    msg += proximity_down << 12;

    uint16_t Airgap = (uint16_t)round((airgap(thermocouple_pressure_airgap_outputBuffer[6]) - AIRGAP_OFFEST) / AIRGAP_FACTOR);
    msg += Airgap << 8;

    uint16_t Pressure = (uint16_t)round((pressure(thermocouple_pressure_airgap_outputBuffer[7]) - PRESSURE_OFFEST) / PRESSURE_FACTOR);
    msg += Pressure << 0;

    return msg;
}

uint64_t hems_temperature_msg(void)
{
    uint64_t msg;
    msg = 0;

    uint16_t thermistor_1 = (uint16_t)round(((0.5 * (thermistor(thermistor_outputBuffer[0]) + thermistor(thermistor_outputBuffer[2]))) - TEMPERATURE_OFFEST) / TEMPERATURE_FACTOR);
    uint16_t thermistor_2 = (uint16_t)round(((0.5 * (thermistor(thermistor_outputBuffer[1]) + thermistor(thermistor_outputBuffer[3]))) - TEMPERATURE_OFFEST) / TEMPERATURE_FACTOR);
    uint16_t thermistor_3 = (uint16_t)round(((0.5 * (thermistor(thermistor_outputBuffer[4]) + thermistor(thermistor_outputBuffer[6]))) - TEMPERATURE_OFFEST) / TEMPERATURE_FACTOR);
    uint16_t thermistor_4 = (uint16_t)round(((0.5 * (thermistor(thermistor_outputBuffer[5]) + thermistor(thermistor_outputBuffer[7]))) - TEMPERATURE_OFFEST) / TEMPERATURE_FACTOR);

    msg += thermistor_1 << 0;
    msg += thermistor_2 << 10;
    msg += thermistor_3 << 20;
    msg += thermistor_4 << 30;

    return msg;
}

uint64_t lems_temperature_msg(void)
{
    uint64_t msg;
    msg = 0;

    uint16_t thermistor_1 = (uint16_t)round(((0.5 * (thermistor(thermistor_outputBuffer[8]) + thermistor(thermistor_outputBuffer[10]))) - TEMPERATURE_OFFEST) / TEMPERATURE_FACTOR);
    uint16_t thermistor_2 = (uint16_t)round(((0.5 * (thermistor(thermistor_outputBuffer[9]) + thermistor(thermistor_outputBuffer[11]))) - TEMPERATURE_OFFEST) / TEMPERATURE_FACTOR);
    uint16_t thermistor_3 = (uint16_t)round(((0.5 * (thermistor(thermistor_outputBuffer[12]) + thermistor(thermistor_outputBuffer[14]))) - TEMPERATURE_OFFEST) / TEMPERATURE_FACTOR);
    uint16_t thermistor_4 = (uint16_t)round(((0.5 * (thermistor(thermistor_outputBuffer[13]) + thermistor(thermistor_outputBuffer[15]))) - TEMPERATURE_OFFEST) / TEMPERATURE_FACTOR);

    msg += thermistor_1 << 0;
    msg += thermistor_2 << 10;
    msg += thermistor_3 << 20;
    msg += thermistor_4 << 30;

    return msg;
}

uint64_t lim_temperature_msg(void)
{
    uint64_t msg;
    msg = 0;

    uint16_t thermistor_1 = (uint16_t)round(((0.5 * (thermistor(thermistor_outputBuffer[16]) + thermistor(thermistor_outputBuffer[18]))) - TEMPERATURE_OFFEST) / TEMPERATURE_FACTOR);
    uint16_t thermistor_2 = (uint16_t)round(((0.5 * (thermistor(thermistor_outputBuffer[17]) + thermistor(thermistor_outputBuffer[19]))) - TEMPERATURE_OFFEST) / TEMPERATURE_FACTOR);
    uint16_t thermistor_3 = (uint16_t)round(((0.5 * (thermistor(thermistor_outputBuffer[20]) + thermistor(thermistor_outputBuffer[22]))) - TEMPERATURE_OFFEST) / TEMPERATURE_FACTOR);
    uint16_t thermistor_4 = (uint16_t)round(((0.5 * (thermistor(thermistor_outputBuffer[21]) + thermistor(thermistor_outputBuffer[23]))) - TEMPERATURE_OFFEST) / TEMPERATURE_FACTOR);
    uint16_t thermistor_5 = (uint16_t)round(((0.5 * (thermistor(thermistor_outputBuffer[24]) + thermistor(thermistor_outputBuffer[26]))) - TEMPERATURE_OFFEST) / TEMPERATURE_FACTOR);
    uint16_t thermistor_6 = (uint16_t)round(((0.5 * (thermistor(thermistor_outputBuffer[25]) + thermistor(thermistor_outputBuffer[27]))) - TEMPERATURE_OFFEST) / TEMPERATURE_FACTOR);
    uint16_t thermistor_7 = (uint16_t)round(((0.5 * (thermistor(thermistor_outputBuffer[28]) + thermistor(thermistor_outputBuffer[30]))) - TEMPERATURE_OFFEST) / TEMPERATURE_FACTOR);
    uint16_t thermistor_8 = (uint16_t)round(((0.5 * (thermistor(thermistor_outputBuffer[29]) + thermistor(thermistor_outputBuffer[31]))) - TEMPERATURE_OFFEST) / TEMPERATURE_FACTOR);

    msg += thermistor_1 << 0;
    msg += thermistor_2 << 5;
    msg += thermistor_3 << 10;
    msg += thermistor_4 << 20;
    msg += thermistor_5 << 25;
    msg += thermistor_6 << 30;
    msg += thermistor_7 << 35;
    msg += thermistor_8 << 40;

    return msg;
}

uint64_t pressure_msg_1(void)
{
    uint64_t msg;
    msg = 0;

    uint16_t pressure_1 = (uint16_t)round((pressure(thermocouple_pressure_airgap_outputBuffer[0]) - PRESSURE_OFFEST) / PRESSURE_FACTOR);
    uint16_t pressure_2 = (uint16_t)round((pressure(thermocouple_pressure_airgap_outputBuffer[1]) - PRESSURE_OFFEST) / PRESSURE_FACTOR);
    uint16_t pressure_3 = (uint16_t)round((pressure(thermocouple_pressure_airgap_outputBuffer[2]) - PRESSURE_OFFEST) / PRESSURE_FACTOR);
    uint16_t pressure_4 = (uint16_t)round((pressure(thermocouple_pressure_airgap_outputBuffer[3]) - PRESSURE_OFFEST) / PRESSURE_FACTOR);

    msg += pressure_1 << 0;
    msg += pressure_2 << 16;
    msg += pressure_3 << 32;
    msg += pressure_4 << 48;

    return msg;
}

uint64_t pressure_msg_2(void)
{
    uint64_t msg;
    msg = 0;

    uint16_t pressure_5 = (uint16_t)round((pressure(thermocouple_pressure_airgap_outputBuffer[4]) - PRESSURE_OFFEST) / PRESSURE_FACTOR);
    uint16_t pressure_6 = (uint16_t)round((pressure(thermocouple_pressure_airgap_outputBuffer[5]) - PRESSURE_OFFEST) / PRESSURE_FACTOR);
    uint16_t pressure_7 = (uint16_t)round((pressure(thermocouple_pressure_airgap_outputBuffer[4]) - PRESSURE_OFFEST) / PRESSURE_FACTOR);
    uint16_t pressure_8 = (uint16_t)round((pressure(thermocouple_pressure_airgap_outputBuffer[5]) - PRESSURE_OFFEST) / PRESSURE_FACTOR);

    msg += pressure_5 << 0;
    msg += pressure_6 << 16;
    msg += pressure_7 << 32;
    msg += pressure_8 << 48;

    return msg;
}

uint64_t pcm_temperature_msg_1(void)
{
    uint64_t msg;
    msg = 0;

    uint16_t pcm_temperature_1 = (uint16_t)round((thermocouple(thermocouple_pressure_airgap_outputBuffer[8]) - TEMPERATURE_OFFEST) / TEMPERATURE_FACTOR);
    uint16_t pcm_temperature_2 = (uint16_t)round((thermocouple(thermocouple_pressure_airgap_outputBuffer[9]) - TEMPERATURE_OFFEST) / TEMPERATURE_FACTOR);
    uint16_t pcm_temperature_3 = (uint16_t)round((thermocouple(thermocouple_pressure_airgap_outputBuffer[10]) - TEMPERATURE_OFFEST) / TEMPERATURE_FACTOR);
    uint16_t pcm_temperature_4 = (uint16_t)round((thermocouple(thermocouple_pressure_airgap_outputBuffer[11]) - TEMPERATURE_OFFEST) / TEMPERATURE_FACTOR);

    msg += pcm_temperature_1 << 0;
    msg += pcm_temperature_2 << 9;
    msg += pcm_temperature_3 << 18;
    msg += pcm_temperature_4 << 27;

    return msg;
}

uint64_t pcm_temperature_msg_2(void)
{
    uint64_t msg;
    msg = 0;

    uint16_t pcm_temperature_5 = (uint16_t)round((thermocouple(thermocouple_pressure_airgap_outputBuffer[12]) - TEMPERATURE_OFFEST) / TEMPERATURE_FACTOR);
    uint16_t pcm_temperature_6 = (uint16_t)round((thermocouple(thermocouple_pressure_airgap_outputBuffer[13]) - TEMPERATURE_OFFEST) / TEMPERATURE_FACTOR);
    uint16_t pcm_temperature_7 = (uint16_t)round((thermocouple(thermocouple_pressure_airgap_outputBuffer[14]) - TEMPERATURE_OFFEST) / TEMPERATURE_FACTOR);
    uint16_t pcm_temperature_8 = (uint16_t)round((thermocouple(thermocouple_pressure_airgap_outputBuffer[15]) - TEMPERATURE_OFFEST) / TEMPERATURE_FACTOR);

    msg += pcm_temperature_5 << 0;
    msg += pcm_temperature_6 << 9;
    msg += pcm_temperature_7 << 18;
    msg += pcm_temperature_8 << 27;

    return msg;
}

CAN_Frame daq_sensors_1_lb(void)
{
    uint64_t msg;
    msg = 0;
    msg = daq_sensors_msg();

    DAQ_Sensors_1_LB.id = DAQ_Sensors_1_LB_id;
    DAQ_Sensors_1_LB.dlc = 2;
    packageData(DAQ_Sensors_1_LB.data, msg, DAQ_Sensors_1_LB.dlc);

    return DAQ_Sensors_1_LB;
}

CAN_Frame daq_sensors_1_rf(void)
{
    uint64_t msg;
    msg = 0;
    msg = daq_sensors_msg();

    DAQ_Sensors_1_RF.id = DAQ_Sensors_1_RF_id;
    DAQ_Sensors_1_RF.dlc = 2;
    packageData(DAQ_Sensors_1_RF.data, msg, DAQ_Sensors_1_RF.dlc);
    
    return DAQ_Sensors_1_RF;
}

CAN_Frame daq_sensors_2_lf(void)
{
    uint64_t msg;
    msg = 0;
    msg = daq_sensors_msg();

    DAQ_Sensors_2_LF.id = DAQ_Sensors_2_LF_id;
    DAQ_Sensors_2_LF.dlc = 2;
    packageData(DAQ_Sensors_2_LF.data, msg, DAQ_Sensors_2_LF.dlc);

    return DAQ_Sensors_2_LF;
}

CAN_Frame daq_sensors_2_rb(void)
{
    uint64_t msg;
    msg = 0;
    msg = daq_sensors_msg();

    DAQ_Sensors_2_RB.id = DAQ_Sensors_2_RB_id;
    DAQ_Sensors_2_RB.dlc = 2;
    packageData(DAQ_Sensors_2_RB.data, msg, DAQ_Sensors_2_RB.dlc);

    return DAQ_Sensors_2_RB;
}

CAN_Frame hems_temperature_rf(void)
{
    uint64_t msg;
    msg = 0;
    msg = hems_temperature();

    HEMS_Temperature_RF.id = HEMS_Temperature_RF_id;
    HEMS_Temperature_RF.dlc = 5;
    packageData(HEMS_Temperature_RF.data, msg, HEMS_Temperature_RF.dlc);

    return HEMS_Temperature_RF;
} 

CAN_Frame hems_temperature_rb(void)
{
    uint64_t msg;
    msg = 0;
    msg = hems_temperature();

    HEMS_Temperature_RB.id = HEMS_Temperature_RB_id;
    HEMS_Temperature_RB.dlc = 5;
    packageData(HEMS_Temperature_RB.data, msg, HEMS_Temperature_RB.dlc);

    return HEMS_Temperature_RB;
}

CAN_Frame hems_temperature_lb(void)
{
    uint64_t msg;
    msg = 0;
    msg = hems_temperature();

    HEMS_Temperature_LB.id = HEMS_Temperature_LB_id;
    HEMS_Temperature_LB.dlc = 5;
    packageData(HEMS_Temperature_LB.data, msg, HEMS_Temperature_LB.dlc);

    return HEMS_Temperature_LB;
}

CAN_Frame hems_temperature_lf(void)
{
    uint64_t msg;
    msg = 0;
    msg = hems_temperature();

    HEMS_Temperature_LF.id = HEMS_Temperature_LF_id;
    HEMS_Temperature_LF.dlc = 5;
    packageData(HEMS_Temperature_LF.data, msg, HEMS_Temperature_LF.dlc);

    return HEMS_Temperature_LF;
}

CAN_Frame lems_temperature_rf(void)
{
    uint64_t msg;
    msg = 0;
    msg = lems_temperature();

    LEMS_Temperature_RF.id = LEMS_Temperature_RF_id;
    LEMS_Temperature_RF.dlc = 5;
    packageData(LEMS_Temperature_RF.data, msg, LEMS_Temperature_RF.dlc);

    return LEMS_Temperature_RF;
}

CAN_Frame lems_temperature_rb(void)
{
    uint64_t msg;
    msg = 0;
    msg = lems_temperature();

    LEMS_Temperature_RB.id = LEMS_Temperature_RB_id;
    LEMS_Temperature_RB.dlc = 5;
    packageData(LEMS_Temperature_RB.data, msg, LEMS_Temperature_RB.dlc);

    return LEMS_Temperature_RB;
}

CAN_Frame lems_temperature_lb(void)
{
    uint64_t msg;
    msg = 0;
    msg = lems_temperature();

    LEMS_Temperature_LB.id = LEMS_Temperature_LB_id;
    LEMS_Temperature_LB.dlc = 5;
    packageData(LEMS_Temperature_LB.data, msg, LEMS_Temperature_LB.dlc);

    return LEMS_Temperature_LB;
}

CAN_Frame lems_temperature_lf(void)
{
    uint64_t msg;
    msg = 0;
    msg = lems_temperature();

    LEMS_Temperature_LF.id = LEMS_Temperature_LF_id;
    LEMS_Temperature_LF.dlc = 5;
    packageData(LEMS_Temperature_LF.data, msg, LEMS_Temperature_LF.dlc);

    return LEMS_Temperature_LF;
}

CAN_Frame lim_temperature_lf(void)
{
    uint64_t msg;
    msg = 0;
    msg = lim_temperature();

    LIM_Temperature_LF.id = LIM_Temperature_LF_id;
    LIM_Temperature_LF.dlc = 5;
    packageData(LIM_Temperature_LF.data, msg, LIM_Temperature_LF.dlc);

    return LIM_Temperature_LF;
}

CAN_Frame lim_temperature_rb(void)
{
    uint64_t msg;
    msg = 0;
    msg = lim_temperature();

    LIM_Temperature_RB.id = LIM_Temperature_RB_id;
    LIM_Temperature_RB.dlc = 5;
    packageData(LIM_Temperature_RB.data, msg, LIM_Temperature_RB.dlc);

    return LIM_Temperature_RB;
}

CAN_Frame thermal_pressure_1(void)
{
    uint64_t msg;
    msg = 0;
    msg = pressure_msg_1();

    Thermal_Pressure_1.id = Thermal_Pressure_1_id;
    Thermal_Pressure_1.dlc = 8;
    packageData(Thermal_Pressure_1.data, msg, Thermal_Pressure_1.dlc);

    return Thermal_Pressure_1;
}

CAN_Frame thermal_pressure_2(void)
{
    uint64_t msg;
    msg = 0;
    msg = pressure_msg_2();

    Thermal_Pressure_2.id = Thermal_Pressure_2_id;
    Thermal_Pressure_2.dlc = 8;
    packageData(Thermal_Pressure_2.data, msg, Thermal_Pressure_2.dlc);

    return Thermal_Pressure_2;
}

CAN_Frame pcm_temperature_1(void)
{
    uint64_t msg;
    msg = 0;
    msg = pcm_temperature_msg_1();

    PCM_Temperature_1.id = PCM_Temperature_1_id;
    PCM_Temperature_1.dlc = 5;
    packageData(PCM_Temperature_1.data, msg, PCM_Temperature_1.dlc);

    return PCM_Temperature_1;
}

CAN_Frame pcm_temperature_2(void)
{
    uint64_t msg;
    msg = 0;
    msg = pcm_temperature_msg_2();

    PCM_Temperature_2.id = PCM_Temperature_2_id;
    PCM_Temperature_2.dlc = 5;
    packageData(PCM_Temperature_2.data, msg, PCM_Temperature_2.dlc);

    return PCM_Temperature_2;
}
