#include "nrf.h"
#include "math.h"

#ifdef NRF51

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "nordic_common.h"
#include "nrf_drv_config.h"
#include "nrf_adc.h"
#include "boards.h"
#include "app_uart.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "nrf_gpiote.h"
#include "nrf_gpio.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_timer.h"
#include "nrf_drv_gpiote.h"
#include "device_manager.h"
#include "ble.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "pstorage.h"
#include "app_trace.h"
#include "bsp.h"
#include "bsp_btn_ble.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "app_pwm.h"

#define BLELED  24                      // shows BLE status
#define BATLED1  5                      // Shows battery life
#define BATLED2  20                     // Shows battery life
#define piezoINTPin 3                   // interrupt pin for the piezo
#define piezoAnalog 4                   // analog pin for piezo
#define audioPin 6                      // analog pin for audio input from amplifier

#define sampleDuration 100              // time between piezo buzzer sampling 20Hz 
#define upperThreshold 100              // threshold value to decide when the detected sound is a knock or not
#define numberOfTaps 5                  // maximum number of taps to turn on/off device
#define knockInterval 500               // maximum time between knocks*/
#define VBAT_MAX_IN_MV 3000             // battery voltage in millivolts

// select a flash page that isn't in use (see Memory.h for more info)
#define  MY_FLASH_PAGE  251

// double level of indirection required to get gcc
// to apply the stringizing operator correctly
#define  str(x)   xstr(x)
#define  xstr(x)  #x

#define NOTE_G3  196
#define NOTE_GS3 208
#define NOTE_A3  220
#define NOTE_AS3 233
#define NOTE_B3  247
#define NOTE_C4  262
#define NOTE_CS4 277
#define NOTE_D4  294
#define NOTE_DS4 311
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_FS4 370
#define NOTE_G4  392
#define NOTE_GS4 415
#define NOTE_A4  440
#define NOTE_AS4 466
#define NOTE_B4  494
#define NOTE_C5  523
#define NOTE_CS5 554
#define NOTE_D5  587
#define NOTE_DS5 622
#define NOTE_E5  659
#define NOTE_F5  698
#define NOTE_FS5 740
#define NOTE_G5  784
#define NOTE_GS5 831
#define NOTE_A5  880
#define NOTE_AS5 932
#define NOTE_B5  988
#define NOTE_C6  1047
#define NOTE_CS6 1109
#define NOTE_D6  1175
#define NOTE_DS6 1245
#define NOTE_E6  1319
#define NOTE_F6  1397
#define NOTE_FS6 1480
#define NOTE_G6  1568
#define NOTE_GS6 1661
#define NOTE_A6  1760
#define NOTE_AS6 1865
#define NOTE_B6  1976

/*
 * the device settings in this order
 * 0 - volume TBD
 * 1 - light intensity (0 - 100)
 * 2 - number of sound bits (0 - 10)
 * 3 - number of light pulses (0 - 20)
 * 4 - loud 1st sound cycle (0/1) TBD
 * 5 - whistle sensor state (0/1) TBD
 * 6 - auto off time (1 - 25[never]) TBD
 * 7 - melody (1- 4)
 * 8 - plays (0 - 2)
 * 9 - whistle 0 (0/1) tbd
 */
struct DeviceSettings{
  uint8_t deviceSettings[10];
};

uint8_t defaultSettings[10] = {100, 100, 2, 8, 0, 0, 1, 1, 0, 0}; // default settings
struct DeviceSettings settings = {100, 100, 2, 8, 1, 0, 1, 3, 1, 0}; 
//struct DeviceSettings *p = (DeviceSettings*)ADDRESS_OF_PAGE(MY_FLASH_PAGE);
bool updateReceived = false;
bool saveUpdates = false;

long onTime = 0; // 
bool whistleDetected = false;
int ledPulseTime = 4;
//(100 * 4)/settings.deviceSettings[1]; // calculate time interval for fade

// variables for BLE LED control
bool BLEConnected = false; // Holds the state of the BLE connection
bool BLEWindow = false; // determines if BLE can be activated
bool BLEActivated = false; // determines if BLE can be activated
long timer = 0; // manages blinking of BLE led
long bleWindowTimer = 0; // holds time duration of BLE window


// variables for piezo control:
volatile long timeTrack = 0; // holds execution time
volatile uint8_t taps = 0; //holds the number of consecutive taps

enum DeviceState{
  ON,
  OFF
}deviceState = OFF;


//variables for playing music on piezo {};//
int ONNote[]= {NOTE_C4, NOTE_G4};
int OFFNote[] = {NOTE_G4, NOTE_C4};//gc
int lullaby0[] = {NOTE_G4, NOTE_A4, NOTE_D5, NOTE_A4,0};// gada palis melody // {NOTE_G4, NOTE_A4, NOTE_D5, NOTE_A4}
int lullaby1[] = {NOTE_AS4, NOTE_C5, NOTE_GS4, NOTE_GS3, NOTE_DS4}; // F#, G#, E, e, H close encounters Bflat C Aflat (octavelower) AFlat E flat
int lullaby2[] = {NOTE_A4, NOTE_A4, NOTE_A4, NOTE_F4, NOTE_C5, NOTE_A4, NOTE_F4, NOTE_C5, NOTE_A4, NOTE_E5, NOTE_E5, NOTE_E5, NOTE_F5, NOTE_C5, NOTE_GS4, NOTE_F4, NOTE_C5, NOTE_A4}; // A,A,A,F,C, A,F,C,A, E,E,E,f,C, G#, F,C,A imperial march
int lullaby3[] = {NOTE_G4, NOTE_G4, NOTE_D5, NOTE_D5, NOTE_E5, NOTE_E5, NOTE_D5, NOTE_C5, NOTE_C5, NOTE_B4, NOTE_B4, NOTE_A4, NOTE_A4, NOTE_G4}; // G,G,D,D,E,E,D,C,C,B,B,A,A,G twinkle twinkle little star
int ONbeats[] = { 1, 2};
int lullabyBeats0[] = {2, 2, 9, 11,2};
int lullabyBeats1[] = {2, 3, 4, 3, 12};
int lullabyBeats2[] = {4, 4, 4, 3, 1, 4, 3, 1, 8, 4, 4, 4, 3, 1, 4, 3, 1, 8};
int lullabyBeats3[] = {2, 2, 2, 2, 2, 2, 5, 2, 2, 2, 2, 2, 2, 4};
int tempo = 150;

// implements LED breathing when lullaby is playing
volatile int tSign = 1;
volatile int fade = 0;
volatile uint8_t fadeTrack = 0;

//variables for signal processing
short rawData[512]; // will hold 512 samples of the signal
short maximum = 0, minimum = 0; // holds the peak and ebb values
short peaks = 0; // number of peaks in the sampled signal
short fullWaves = 0; //number of full waves of the signal derived from the number of peaks
float frequency = 0; // frequency derived from (fullWaves*1000ms)/sampling time in ms
uint8_t frequencyMonitor = 0; // counts the number of times the frequency supasses 244Hz
uint8_t sampleNumber = 0; // holds number of samples
volatile bool babyCrying = false;
long monitorTiming = 0; //records time of last positive frequency detection
int total = 0;

enum signalState{
  rising,
  falling,
}edge;

volatile static uint32_t millis = 0;


ret_code_t err_code = 0;

/*********************************************************prototypes******************************************************/
bool  FFT(float *Rdat, float *Idat, int N, int LogN);                          // fft function
uint16_t maxFreqAnalysis(float * arr, uint16_t size);                          // function that analyses the sampled frequencies
void isBabyCrying(void);                                                       // function detects whether a baby is crying or not
void ADC_IRQHandler(void);                                                     // samples analog readigns and fills arrays with the data
void adcConfig(void);                                                          // configure the ADC
static void uart_init(void);                                                   // configuring the UART parameters
void uart_error_handle(app_uart_evt_t * p_event);                              // handling data received in UART
void timer1Setup(unsigned int ms);                                             // sets up timer interrupt
void attachInterrupt(uint8_t singlePin, uint8_t pin, uint8_t state);           // function to set hardware interrupts
void in_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action);   // hardware interrupt for detecting knocks
void RTC1Setup(void);                                                          // sets up RTC1 to track timer in ms
void RTC1_IRQHandler(void);                                                    // interrupt handler that handles increment of time every ms
void changePinStates(bool x);                                                  // changes conficuration of piezo pins
void turnOn(void);                                                             // turns on device features after waking up
void sendToSleep(void);                                                        // does some houskeeping before sending device into deep sleep
static void sleep_mode_enter(void);                                            // sends device into deep sleep
void knockDetect(void);                                                        // listens to number of knocks/taps on the piezo
void playTone(int tone, int duration, int led);                                // play the tone selected by the user
void playNote(int note, int duration, int led);                                // play note seleected by user
void onOffTone(int index);                                                     // play the on/off tone
void test(void);                                                               // play a test tone
void pwm_ready_callback(uint32_t pwm_id);                                      // PWM callback function
void PWM_config(void);                                                         // PWM configuration file
void analogWrite(uint8_t channel, uint8_t value);                              // Function that performs PWM
uint8_t battery_level_get(void);                                               // gets the battery level
void batteryLevelCheck(void);                                                  // blinks battery indicator LEDs
void lullabySong(uint8_t i);                                                   // sets the lullaby that is going to play
void playLullaby(void);                                                        // controls the way the lullaby plays
void TIMER2_IRQHandler(void);                                                  // timer2 IRQ handler
/*************************************************************************************************************************/


/**************************************audio sampling variables and constants*********************************************/

//#define ENABLE_LOOPBACK_TEST  /**< if defined, then this example will be a loopback test, which means that TX should be connected to RX to get data loopback. */

#define MAX_TEST_DATA_BYTES     (15U)                /**< max number of test bytes to be used for tx and rx. */
#define UART_TX_BUF_SIZE 256                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 1                           /**< UART RX buffer size. */

#ifndef NRF_APP_PRIORITY_HIGH
#define NRF_APP_PRIORITY_HIGH                   1
#endif

//// config FFT parameters
#define ARRAY_SIZE                              64                              // the size of FFT
#define LOGN_ARRAY_SIZE                         6
#define BIN_STEP                                50000/ARRAY_SIZE/2
//// ATTENTION! It's important to configure fft offset below. 
//// This is the adc value you get in total silence in float. 
//// My mic have few volts offset do this value is 172
#define FFT_OFFSET                              170.0                           // value to distract 
//// Cry detection sensivity parameters
#define AVER_POWER                              4                               // average power of signal shoulâ be higher
#define HAPR                                    25                              // harmonic to average power ratio of signal should be higher
#define LOW_BAND                                700                             // pitch freq of signal should be between 700 Hz ...
#define HIGH_BAND                               4000                            // ... and 4000 Hz
#define SILENCE_TRSHLD                          5                               // signal with power lower than this considered as silence
#define NUM_SILENT_SAMPLES                      15                              // if it's a baby's cry there should more than 15 silent samples

volatile bool data1Ready = false, data2Ready = false;
float inArrayR[2][ARRAY_SIZE] = {0};                                            // vars to process FFT 
float inArrayI[ARRAY_SIZE] = {0};                                               // vars to process FFT 
float analogSample = 0;																													// holds the analog value read from the amplifier
static bool cryDetected = false;
volatile bool silence = true;
/*************************************************************************************************************************/

/***************************************************PWM variables*********************************************************/
APP_PWM_INSTANCE(PWM1,1);                   // Create the instance "PWM1" using TIMER1.
static volatile bool ready_flag;            // A flag indicating PWM status.
/*************************************************************************************************************************/

/**********************************************************NRF based functions********************************************************/
// ADC code
/**@brief   Function for configuring up ADC.
 *
 * @details ADC configured to read from pin 6 with a sample rate of 50kHz.
 *          Analog samples are filled in 2 arrays for analysis
 *          
 */
/**@snippet [configuring the ADC] */
void adcConfig(void) {
    const nrf_adc_config_t nrf_adc_config = { 
      NRF_ADC_CONFIG_RES_8BIT,                                             // 20 us ack time -> 50kHz sample rate
      NRF_ADC_CONFIG_SCALING_INPUT_TWO_THIRDS, 
      NRF_ADC_CONFIG_REF_SUPPLY_ONE_HALF 
    };
    nrf_adc_configure( (nrf_adc_config_t *)&nrf_adc_config);
    nrf_adc_input_select(NRF_ADC_CONFIG_INPUT_7);                          // read from pin 6
    nrf_adc_int_enable(ADC_INTENSET_END_Enabled << ADC_INTENSET_END_Pos);
    NVIC_SetPriority(ADC_IRQn, NRF_APP_PRIORITY_HIGH);
    NVIC_EnableIRQ(ADC_IRQn);
}

/**@snippet [ADC handler fills 2 arrays with data] */
void ADC_IRQHandler(void) {
  static uint32_t sampleCount = 0;
  nrf_adc_conversion_event_clean();
  if (data1Ready == false || data2Ready == false) {
		analogSample = (float)nrf_adc_result_get();
		//silence = (analogSample - FFT_OFFSET > 5)? false : true;
    inArrayR[0][sampleCount] = (float)analogSample - FFT_OFFSET;
    sampleCount++;
    if (sampleCount >= ARRAY_SIZE) {
      data1Ready = true;                // first frame ready to analyze
    }
    if (sampleCount >= ARRAY_SIZE * 2) {
      data2Ready = true;                // second frame ready to analyze
      sampleCount = 0;
    }
  }   
  nrf_adc_start();                      // trigger next ADC conversion
}


// UART code
/**@brief   Function for handling app_uart events.
 *
 * @details This function will receive a single character from the app_uart module and append it to 
 *          a string. The string will be be sent over BLE when the last character received was a 
 *          'new line' i.e '\n' (hex 0x0D) or if the string has reached a length of 
 *          @ref NUS_MAX_DATA_LENGTH.
 */
/**@snippet [Handling the data received over UART] */
void uart_error_handle(app_uart_evt_t * p_event){
    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_communication);
    }
    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_code);
    }
}
/**@snippet [Handling the data received over UART] */


/**@brief  Function for initializing the UART module.
 */
/**@snippet [UART Initialization] */
static void uart_init(void){
    uint32_t                     err_code;
    const app_uart_comm_params_t comm_params =
      {
          RX_PIN_NUMBER,
          TX_PIN_NUMBER,
          RTS_PIN_NUMBER,
          CTS_PIN_NUMBER,
          APP_UART_FLOW_CONTROL_ENABLED,
          false,
          UART_BAUDRATE_BAUDRATE_Baud38400
      };

    APP_UART_FIFO_INIT(&comm_params,
                         UART_RX_BUF_SIZE,
                         UART_TX_BUF_SIZE,
                         uart_error_handle,
                         APP_IRQ_PRIORITY_LOW,
                         err_code);

    APP_ERROR_CHECK(err_code);
}
/**@snippet [UART Initialization] */

// Timer interrupt code
/**@brief   Function for setting up a timer interrupt.
 *
 * @details This function will setup a timer interrupt using timer 2
 *          The frequency of the interrupt will be defined by the variable ms that you pass into the function
 *          In this application the timer will have a frequency of 1kHz and will be used to control the BAT LEDs
 *          This means that we will pass a value 1 into ms. 
 *          (ms stands for milliseconds so passing 1 means the timer interrupt is set at 1kHz. setting 10 means its runnign at 100Hz)
 *          
 */
/**@snippet [Setting up the timer] */
void timer2Setup(unsigned int ms){                                 // directly pass the value you want the cycle to be in mS
  NRF_TIMER2->TASKS_STOP = 1;                                      // Stop timer
  NRF_TIMER2->MODE = TIMER_MODE_MODE_Timer;                        // sets the timer to TIME mode (doesn't make sense but OK!)
  NRF_TIMER2->BITMODE = TIMER_BITMODE_BITMODE_16Bit;               // with BLE only Timer 1 and Timer 2 and that too only in 16bit mode
  NRF_TIMER2->PRESCALER = 9;                                       // Prescaler 9 produces 31250 Hz timer frequency => t = 1/f =>  32 uS
                                                                   // The figure 31250 Hz is generated by the formula (16M) / (2^n)
                                                                   // where n is the prescaler value
                                                                   // hence (16M)/(2^9)=31250
  NRF_TIMER2->TASKS_CLEAR = 1;                                     // Clear timer
 
  //-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  //        Conversion to make cycle calculation easy
  //        Since the cycle is 32 uS hence to generate cycles in mS we need 1000 uS
  //        1000/32 = 31.25  Hence we need a multiplication factor of 31.25 to the required cycle time to achive it
  //        e.g to get a delay of 10 mS      we would do
  //        NRF_TIMER2->CC[0] = (10*31)+(10/4);
  //-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 
  NRF_TIMER2->CC[0] = (ms * 31) + (ms / 4);                                                                                  //CC[0] register holds interval count value i.e your desired cycle
  NRF_TIMER2->INTENSET = TIMER_INTENSET_COMPARE0_Enabled << TIMER_INTENSET_COMPARE0_Pos;                                     // Enable COMAPRE0 Interrupt
  NRF_TIMER2->SHORTS = (TIMER_SHORTS_COMPARE0_CLEAR_Enabled << TIMER_SHORTS_COMPARE0_CLEAR_Pos);                             // Count then Complete mode enabled
  NVIC_EnableIRQ(TIMER2_IRQn);                                                                            // also used in variant.cpp in the RFduino2.2 folder to configure the RTC1                                                                                              // Start TIMER
}


/**@breif Configure hardware interrupts */


/**@details  Sets hardware interrupt on the pin that is passed to the function using the pin variable. 
 *           The interrupt can be on the single pin or on the port, depending on the value of singlePin variable
 *           Set singlePin as 0 to use a port event and 1 to use a dedicated event
 */
/**@snippet [Hardware interrupt set up] */
void attachInterrupt(uint8_t singlePin, uint8_t pin, uint8_t state){
		if(state){
			//Initialize gpiote module
			err_code = nrf_drv_gpiote_init();
			APP_ERROR_CHECK(err_code);
			//Configure sense input pin to enable wakeup and interrupt on button press.
			nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_LOTOHI(singlePin);    //Configure to generate interrupt and wakeup on pin signal low. "false" means that gpiote will use the PORT event, which is low power, i.e. does not add any noticable current consumption (<<1uA). Setting this to "true" will make the gpiote module use GPIOTE->IN events which add ~8uA for nRF52 and ~1mA for nRF51.
			in_config.pull = NRF_GPIO_PIN_NOPULL;                                               //Configure pullup for input pin to prevent it from floting. Pin is pulled down when button is pressed on nRF5x-DK boards, see figure two in http://infocenter.nordicsemi.com/topic/com.nordic.infocenter.nrf52/dita/nrf52/development/dev_kit_v1.1.0/hw_btns_leds.html?cp=2_0_0_1_4		
			err_code = nrf_drv_gpiote_in_init(pin, &in_config, in_pin_handler);                 //Initialize the pin with interrupt handler in_pin_handler
			APP_ERROR_CHECK(err_code);                                                          //Check potential error
			nrf_drv_gpiote_in_event_enable(pin, true);                                          //Enable event and interrupt for the wakeup pin                                                          //Enable interrupts
		}
		else{
			nrf_drv_gpiote_in_event_enable(pin, false);
		}
}


/** @brief Procedure that keeps track of the current milliseconds
 *   
 *  @details RTC1 is used to provide the time tracking. Its counter increments 
 *					 with a frequency of 1kHz derived from the LFCLK. The variable millis
 *					 is used to track the increments and its value used to keep track of 
 *					 time in milliseconds. The resolution is 1ms. The number of milliseconds 
 *           since the device started are stored on a vaiable called millis.
 */
/** @snipet Function that configures RTC1*/
void RTC1Setup(){                                
		// Internal 32kHz RC
		NRF_CLOCK->LFCLKSRC = CLOCK_LFCLKSRC_SRC_Xtal << CLOCK_LFCLKSRC_SRC_Pos;
		// Start the 32 kHz clock, and wait for the start up to complete
		NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;
		NRF_CLOCK->TASKS_LFCLKSTART = 1;
		while(NRF_CLOCK->EVENTS_LFCLKSTARTED == 0);

		// Configure the RTC to run at 1 millisecond intervals, and make sure COMPARE0 generates an interrupt (this will be the wakeup source)
		NRF_RTC1->PRESCALER = 32;                                                   // set prescaler of 32 to provide 1kHz resolution (32.768kHz/(prescaler + 1))
		NRF_RTC1->EVTENSET = RTC_EVTEN_COMPARE0_Msk; 																// enable event routing
		NRF_RTC1->INTENSET = RTC_INTENSET_COMPARE0_Msk;                             // enable interrupt
		NRF_RTC1->CC[0] = 1;																												// Count to 1, produces interrupt every millisecond
		NVIC_EnableIRQ(RTC1_IRQn);																									// attach the Interrupt handle to the event
		NVIC_SetPriority(RTC1_IRQn, 17);																					  // set IRQ priority
		NRF_RTC1->TASKS_START = 1;                                                  // start the timer
}

/** @snipet [Timer ISR function]*/
void RTC1_IRQHandler(){
	  // NORDIC: CLEAR TASK AS QUICKLY AS POSSIBLE
    NRF_RTC1->TASKS_CLEAR = 1;
    // This handler will be run after wakeup from system ON (RTC wakeup)
    if(NRF_RTC1->EVENTS_COMPARE[0])
    {
				NRF_RTC1->EVENTS_COMPARE[0] = 0;
        millis++;
    }
}

/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void){
    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    NRF_POWER->SYSTEMOFF = 1;
}


/** @brief Procedures that configures and controls PWM
 *   
 *  @details PWM is derived from TIMER1
 */
/** @snipet [PWM config]*/
void PWM_config(){
	  ret_code_t err_code;
	  /* 2-channel PWM, 200Hz, output on pins 20 and 5. */
    app_pwm_config_t pwm1_cfg = APP_PWM_DEFAULT_CONFIG_2CH(5000L, BATLED1, BATLED2);
    
    /* Switch the polarity of the channel. */
    pwm1_cfg.pin_polarity[1] = APP_PWM_POLARITY_ACTIVE_HIGH;
    pwm1_cfg.pin_polarity[0] = APP_PWM_POLARITY_ACTIVE_HIGH;
    
    /* Initialize and enable PWM. */
		err_code = app_pwm_init(&PWM1,&pwm1_cfg,pwm_ready_callback);
    APP_ERROR_CHECK(err_code);
    app_pwm_enable(&PWM1);
}

/** @snipet [PWM call back function]*/
void pwm_ready_callback(uint32_t pwm_id){    // PWM callback function
    ready_flag = true;
}

/** @snipet [Function that performs PWM]*/
void analogWrite(uint8_t channel, uint8_t value){
    /* Set the duty cycle - keep trying until PWM is ready... */
    while (app_pwm_channel_duty_set(&PWM1, channel, value) == NRF_ERROR_BUSY);
}

/** @brief Procedures gets the battery voltage
 *   
 *  @details Battery voltage is calculated by first setting the analog reference to VBG
 *           VBG (internal 1.2 V band gap reference). 
 */
/** @snipet [get battery level]*/
uint8_t battery_level_get(void){
		// Configure ADC
		NRF_ADC->CONFIG     = (ADC_CONFIG_RES_8bit                        << ADC_CONFIG_RES_Pos)     |
													(ADC_CONFIG_INPSEL_SupplyOneThirdPrescaling << ADC_CONFIG_INPSEL_Pos)  |
													(ADC_CONFIG_REFSEL_VBG                      << ADC_CONFIG_REFSEL_Pos)  |
													(ADC_CONFIG_PSEL_Disabled                   << ADC_CONFIG_PSEL_Pos)    |
													(ADC_CONFIG_EXTREFSEL_None                  << ADC_CONFIG_EXTREFSEL_Pos);
		NRF_ADC->EVENTS_END = 0;
		NRF_ADC->ENABLE     = ADC_ENABLE_ENABLE_Enabled;

		NRF_ADC->EVENTS_END  = 0;    // Stop any running conversions.
		NRF_ADC->TASKS_START = 1;

		while (!NRF_ADC->EVENTS_END)
		{
		}

		uint16_t vbg_in_mv = 1200;
		uint8_t adc_max = 255;
		uint16_t vbat_current_in_mv = (NRF_ADC->RESULT * 3 * vbg_in_mv) / adc_max;

		NRF_ADC->EVENTS_END     = 0;
		NRF_ADC->TASKS_STOP     = 1;

		return (uint8_t) ((vbat_current_in_mv * 100) / VBAT_MAX_IN_MV);
}
/**********************************************************************************************************************************************/



// NAME:          FFT.
// PARAMETERS:  
//    float *Rdat    [in, out] - Real part of Input and Output Data (Signal or Spectrum)
//    float *Idat    [in, out] - Imaginary part of Input and Output Data (Signal or Spectrum)
//    int    N       [in]      - Input and Output Data length (Number of samples in arrays)
//    int    LogN    [in]      - Logarithm2(N)
// RETURN VALUE:  false on parameter error, true on success.
//_________________________________________________________________________________________
// NOTE: In this algorithm N and LogN can be only:
//       N    = 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048, 4096, 8192, 16384;
//       LogN = 2, 3,  4,  5,  6,   7,   8,   9,   10,   11,   12,   13,    14;
//_________________________________________________________________________________________
// FFT & ADC CONFIGURATION:
// sample rate - 50000 Hz (from ADC configuration)
// bins - 50000/ARRAY_SIZE/2 - 195,3125 Hz if ARRAY_SIZE = 128
// bin1: -97,65625..97,65625  Hz (center 0 Hz)
// bin2: 97,65625 ..292,96875 Hz (center 195,3125)
// bin3: 292,96875..488,28125 Hz
// bin4: 488,28125..683,59375 Hz
// bin5...
#define  NUMBER_IS_2_POW_K(x)   ((!((x)&((x)-1)))&&((x)>1))  // x is pow(2, k), k=1,2, ...

bool  FFT(float *Rdat, float *Idat, int N, int LogN) {
  // parameters error check:
  /*if((Rdat == NULL) || (Idat == NULL))                  return false;
  if((N > 16384) || (N < 1))                            return false;
  if(!NUMBER_IS_2_POW_K(N))                             return false;
  if((LogN < 2) || (LogN > 14))                         return false;*/

  register int  i, j, n, k, io, ie, in, nn;
  float         ru, iu, rtp, itp, rtq, itq, rw, iw, sr;
  
  static const float Rcoef[14] =
  {  -1.0000000000000000F,  0.0000000000000000F,  0.7071067811865475F,
      0.9238795325112867F,  0.9807852804032304F,  0.9951847266721969F,
      0.9987954562051724F,  0.9996988186962042F,  0.9999247018391445F,
      0.9999811752826011F,  0.9999952938095761F,  0.9999988234517018F,
      0.9999997058628822F,  0.9999999264657178F
  };
  static const float Icoef[14] =
  {   0.0000000000000000F, -1.0000000000000000F, -0.7071067811865474F,
     -0.3826834323650897F, -0.1950903220161282F, -0.0980171403295606F,
     -0.0490676743274180F, -0.0245412285229122F, -0.0122715382857199F,
     -0.0061358846491544F, -0.0030679567629659F, -0.0015339801862847F,
     -0.0007669903187427F, -0.0003834951875714F
  };
  
  nn = N >> 1;
  ie = N;
  for(n=1; n<=LogN; n++) {
    rw = Rcoef[LogN - n];
    iw = Icoef[LogN - n];
    in = ie >> 1;
    ru = 1.0F;
    iu = 0.0F;
    for(j=0; j<in; j++) {
      for(i=j; i<N; i+=ie) {
        io       = i + in;
        rtp      = Rdat[i]  + Rdat[io];
        itp      = Idat[i]  + Idat[io];
        rtq      = Rdat[i]  - Rdat[io];
        itq      = Idat[i]  - Idat[io];
        Rdat[io] = rtq * ru - itq * iu;
        Idat[io] = itq * ru + rtq * iu;
        Rdat[i]  = rtp;
        Idat[i]  = itp;
      }
      sr = ru;
      ru = ru * rw - iu * iw;
      iu = iu * rw + sr * iw;
    }
    ie >>= 1;
  }
  for(j=i=1; i<N; i++) {
    if(i < j) {
      io       = i - 1;
      in       = j - 1;
      rtp      = Rdat[in];
      itp      = Idat[in];
      Rdat[in] = Rdat[io];
      Idat[in] = Idat[io];
      Rdat[io] = rtp;
      Idat[io] = itp;
    }
    k = nn;
    while(k < j){
      j   = j - k;
      k >>= 1;
    }

    j = j + k;
  }
  return true;
}

/**@brief   Function for anaysing the sampled frequencies.
 */
uint16_t maxFreqAnalysis(float * arr, uint16_t size) {
  float maxValNum = arr[0];
  uint16_t maxValIndex = 0;
  
  for (uint16_t i = 1; i < size; i++) {
    if (arr[i] > maxValNum) {
      maxValNum = arr[i];
      maxValIndex = i;
    }
  }
  return maxValIndex;
}

/**@brief   Function for detecting the characteristics of a baby's cry
 */
void isBabyCrying(){
		if (data1Ready == true || data2Ready == true) {
      //// part to check how FFT works
      /*float  p = 2 * 3.141592653589 / ARRAY_SIZE;              
      for(uint16_t i=0; i<ARRAY_SIZE; i++)
      {
        inArrayR[][i] = sin(p * i);                               // create test signal
        inArrayI[i] = 0.0;                                      
      }*/
      uint8_t arrNum;
      if (data1Ready == true) arrNum = 0;
      else arrNum = 1;
      
      //// FFT and signal analysis
      for(uint16_t i=0; i<ARRAY_SIZE; i++) inArrayI[i] = 0.0;         
      FFT(&inArrayR[arrNum][0], &inArrayI[0], ARRAY_SIZE, LOGN_ARRAY_SIZE);             // calc real and im parts
      for(uint16_t i = 0; i < ARRAY_SIZE/2; i++) {
        inArrayR[arrNum][i] = sqrt(inArrayR[arrNum][i]*inArrayR[arrNum][i]+inArrayI[i]*inArrayI[i]);    // calc energy
      }
      
      //// DSP analysis
      static uint16_t maxFreqIndex;
      maxFreqIndex = maxFreqAnalysis(&inArrayR[arrNum][0], ARRAY_SIZE/2);
      
      static uint16_t maxFreq;
      maxFreq = maxFreqIndex * BIN_STEP;
      
      static float maxFreqPower;
      maxFreqPower = inArrayR[arrNum][maxFreqIndex];
        
      static float averPower;
      averPower = 0;
      for(uint16_t i = 0; i < ARRAY_SIZE/2; i++) averPower += inArrayR[arrNum][i];
      averPower = ((averPower - maxFreqPower) / (ARRAY_SIZE/2 - 1));
      
      static uint16_t hapr;
      if (averPower < 0.1) hapr = 0; 
      else hapr = (uint16_t)(((float)maxFreqPower/(float)averPower)*10.0);
      
      // statistics collection
			#define STAT_ARRAY_SIZE         256            // size of data array we want to analyze
			#define ARRAY_SIZE_2POW         8      
      static uint16_t dataCounter = 0; 
      static uint16_t maxFreqArr[STAT_ARRAY_SIZE], maxFreqPowerArr[STAT_ARRAY_SIZE], 
                      averPowerArr[STAT_ARRAY_SIZE], haprArr[STAT_ARRAY_SIZE], silArr[STAT_ARRAY_SIZE];
      
      maxFreqArr[dataCounter] = maxFreq;
      maxFreqPowerArr[dataCounter] = (uint16_t) maxFreqPower;
      averPowerArr[dataCounter] = (uint16_t) averPower;
      haprArr[dataCounter] = hapr;
      if (averPower < SILENCE_TRSHLD) silArr[dataCounter] = 1;
      else silArr[dataCounter] = 0;
      
      // perform some calculations
      if(++dataCounter > (STAT_ARRAY_SIZE-1)) dataCounter = 0;
      static int32_t maxFreqTt;
      static int32_t maxFreqPowerTt;
      static int32_t averPowerTt;
      static int32_t haprTt;
      static int16_t silTt;
      maxFreqTt = maxFreqPowerTt = averPowerTt = haprTt = silTt = 0;
      
      for (uint16_t i = 0; i < STAT_ARRAY_SIZE; i++) {
        maxFreqTt += maxFreqArr[i];
        maxFreqPowerTt += maxFreqPowerArr[i];
        averPowerTt += averPowerArr[i];
        haprTt += haprArr[i];
        silTt += silArr[i];
      }
      maxFreqTt >>= ARRAY_SIZE_2POW;
      maxFreqPowerTt >>= ARRAY_SIZE_2POW;
      averPowerTt >>= ARRAY_SIZE_2POW;
      haprTt >>= ARRAY_SIZE_2POW;
      
      // let's compare calc results with the pattern
			cryDetected = false;
      static uint8_t cryDetectedTimes = 0;                                 
      if (haprTt > HAPR && averPowerTt > AVER_POWER && (maxFreqTt > LOW_BAND && maxFreqTt < HIGH_BAND) && silTt > NUM_SILENT_SAMPLES) {
        if (cryDetectedTimes < 200) cryDetectedTimes++;
      }
      else {
        if (cryDetectedTimes > 0) cryDetectedTimes--;
      }
      if (cryDetectedTimes > 100){
				cryDetected = true;
				//printf("%d", cryDetected);
				//printf("\n");
			}
      else{
				cryDetected = false;
				//printf("%d", cryDetected);
				//printf("\n");
			}
      if (arrNum == 0) data1Ready = false;
      else data2Ready = false;
		}
}


/**@breif Detecting knocks on the device */


/**@details  IRQ for detecting when the piezo is knocked. 
 *           Hardware interrupt on pin 3
 */
/**@snippet Hardware interrupt IRQ */
void in_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action){
	 if((millis - timeTrack) >= sampleDuration){
			//if(deviceState == OFF)
		  taps++;              // increment number of taps
			timeTrack = millis;  // record time of tap
	 }
}






/** @brief Procedure that changes the pin states for piezo control
 *  @details  changes the piezo buzzer pin states and enables/disables 
 *            interrupts on those pins depending on the value in variable x passed into the function. 
 */
/**@snippet Hardware interrupt IRQ */
void changePinStates(bool x){
  if(x){
		nrf_gpio_cfg_input(piezoAnalog, NRF_GPIO_PIN_NOPULL);        // set piezo pin 4 as input
		__enable_irq();                                              // enables all interrupts
    //printf(">>Enable interrupt on pin 3\n");
  }
  else{
		nrf_gpio_cfg_output(piezoAnalog);                            // set piezo pin 4 as output to play music
		__disable_irq();                                             // disables all interrupts
    //printf(">>Disable interrupt on pin 3\n");                                      
  }
}  

/** @brief Procedure blinks battery indicator LEDs 
 *  @details  The procedure controls the number of times the battery indicator LEDs will blink 
 *            depending on the battery voltage. Since the battery is usually 2V (65% of voltage) when the charge 
 *						gets depleted, the battery level will be calculated between the range of 65% to 100%
 */
/**@snippet [Blink battery indicator LEDs] */
void batteryLevelCheck(){
  uint8_t x = battery_level_get();                     // read battery voltage level in percentage in the range of 0 to 3 V
	x = x - 65;                                          // minus 65% since the range that will be used by the indicator is 65% to 100% i.e. 2V to 3V
  int num = (x * 10) / (100 - 65);                     // calculate number of times LEDs should blink
  printf("Blinks: "); printf("%d\n", num); 
	for(uint8_t i = 0; i <= num; i++){                    // blink the Battery indicator LEDs
		analogWrite(0, settings.deviceSettings[1]);        // glow leds with set brightness
		analogWrite(1, settings.deviceSettings[1]);
    nrf_delay_ms(200); 
    analogWrite(0, 0);                                 // turn off leds
		analogWrite(1, 0);
    nrf_delay_ms(200);
  }
}


/** @brief Procedure that wakes up module from deep sleep
 *   
 *  @details This function plays the on tone and wakes the device from deep sleep
 *					 The module is woken up by input on pin 4 
 */
/** @snipet Wake up module*/
void turnOn(){
  for(uint8_t i = 0; i <= settings.deviceSettings[1]; i++){ // glow LED
    analogWrite(0, i); 
		analogWrite(1, i);
    nrf_delay_ms(4);
  }        
  if(settings.deviceSettings[0] > 0){												// if volume is not set to 0
    changePinStates(0);   																	// set pin 2 as output to allow piezo to play and disable interrupts
    onOffTone(1); 																					// sound startup tone
    changePinStates(1); 																		// return pin 2 to INPUT and enable interrupts 
  } 
  nrf_delay_ms(1000);
  batteryLevelCheck(); 																			// indicate the battery voltage on the indicator LEDs
	adcConfig();                            								  // reconfigure ADC so that it can start sampling audio
	nrf_adc_start();                        									// start ADC conversion
  deviceState = ON; 																				// indicate device is ON
  taps = 0; 																								// initialize the number of taps to zero
  BLEWindow = true; 																				// start the BLE window
  bleWindowTimer = millis; 																	// mark time BLE window opens
  onTime = millis / 1000; 																	// mark time device comes On
  printf(">>Device on\n");
  printf(">>BLE window: ");printf("%d\n", BLEWindow);
}

/** @brief Procedure that sends module to sleep
 *   
 *  @details This function ends the bluetooth connection, plays the off tone
 *					 and sends the module to sleep. The module is woken up by input on pin 4 
 */
/** @snipet Force module to enter deep sleep*/
void sendToSleep(){
  //stopBLE();
  for(uint8_t i = settings.deviceSettings[1]; i > 0; i--){     // fade out LED
    analogWrite(0, i); 
		analogWrite(1, i);
    nrf_delay_ms(4);
  }   
  if(settings.deviceSettings[0] > 0){
    changePinStates(0);                                       // set pin 2 as output to allow piezo to play and disable interrupts
    onOffTone(0);                                             // sound shutdonwn tone
    changePinStates(1);                                       // return pin 2 to INPUT and enable interrupts
  }     
  deviceState = OFF;                                          // indicate that device has gone to deep sleep
  taps = 0;                                                   // initialize the number of taps to zero
  //save();                                                   // save available updated settings
  printf(">>Device shutdown\n");
  nrf_delay_ms(2000);
	sleep_mode_enter();                                         // enter deep sleep
}


/** @brief Procedure that processes taps on the device
 *   
 *  @details This function checks to see how many taps have been made 
 *					 5 taps when device is OFF, plays ON tune and wakes up device
 *					 5 taps when device is ON, plays OFF tune and puts the device into deep sleep
 * 					 5 taps when the device is in the BLE window turns on the bluetooth
 */
/** @snipet Control device using taps*/
void knockDetect(){
  if(!BLEWindow){
    if (taps >= numberOfTaps && deviceState == ON){ // 5 taps while device is ON
      printf(">>5 taps detected\n");
      sendToSleep(); // turn off device
    }
    else if (taps >= numberOfTaps - 3 && deviceState == OFF){ // 5 taps while device is OFF
      printf(">>5 taps detected\n");
      turnOn(); // turn on device
    }
  }
  else{
    if(millis - bleWindowTimer < 10000){
      //BLE window is open. User can activate BLE through tapping 5 times
      if(taps >= numberOfTaps-1){
        // BLE started after 5 taps
        BLEConnected = false; // no phone is connected. Should start blinking
        taps = 0; // initialize the number of taps to zero
        BLEWindow = false; // exit the BLE window
        bleWindowTimer = millis; // record time BLE was activated
        timer = 0;
        //startBLE();
        //pinMode(piezoAnalog, OUTPUT);
				printf(">>BLE activated!\n");
        printf(">>BLE activation window closed!\n");
        return;
      }
    }
    else if((millis - bleWindowTimer > 10000) && (millis - bleWindowTimer < 30000)){
      // if the BLE window is open for more than 10 seconds it closes preventing
      // user from activating BLE using taps
      BLEWindow = false;                                      // close BLE window
      taps = 0;                                               // initialize the number of taps to zero
      printf(">>BLE activation window closed!\n");
      return;
    }
  }
}


/** @brief Procedure that plays a tone on the Piezo
 *   
 *  @details This function plays the selected tone on the piezo buzzer by toggling the pizo pins
 *					 low and high using the time delays assigned to each tone. The volume is also adjusted 
 *					 depending on the settings provided by the user
 * 					 
 */
/** @snipet [Play selected tone]*/
void playTone(int tone, int duration, int led) {
  uint8_t x = 0;
  if(led){
    if(fade == 0 || fade == 255)x = 20;
    else x = 7;
  }
  //volume adjustment
  float volume = (float)((float)(settings.deviceSettings[0] * 45) / (float)100);         //map(settings.deviceSettings[0], 0, 100, 0, 45)
  volume /= 1000;
  int delay1 = ((float)(tone)* 2) * (1 - volume);
  int delay2 = ((float)(tone)* 2) * volume;
  if(delay1 > x)delay1 = delay1 - x;
	for (long i = 0; i < duration * 1000L; i += tone * 2) {
    nrf_gpio_pin_clear(piezoAnalog);
		nrf_delay_us(delay1);
		nrf_gpio_pin_set(piezoAnalog);
    nrf_delay_us(delay2); //tone-x
    //if(led)analogWrite(BATLED,fade);
  }
}

/** @snipet [Play note]*/
void playNote(int note, int duration, int led) {
  double c = (float)(1)/(float)(note);
  c *=1000000;
  c/=2;
  playTone((int)(c), duration,led);
}

/** @brief Procedure that plays the on or off note
 *   
 *  @details This function plays the on note when the index passed to it is 1 and plays the off note
 *					 when the index passed to it is 0
 * 					 
 */
/** @snipet [Play the or off note]*/
void onOffTone(int index){
  if(index == 1){
    for (int i = 0; i < 2; i++) {
      if (ONNote[i] == 0) {
        nrf_delay_ms(ONbeats[i] * tempo); // rest
      } else {
        playNote(ONNote[i], ONbeats[i] * 100, 0);
      }
      // pause between notes
      nrf_delay_ms(tempo / 2); 
    }
  }
  else if(index == 0){
    for (int i = 0; i < 2; i++) {
      if (OFFNote[i] == 0) {
        nrf_delay_ms(ONbeats[i] * tempo); // rest
      } else {
        playNote(OFFNote[i], ONbeats[i] * 100, 0);
      }
      // pause between notes
      nrf_delay_ms(tempo / 2); 
    }
  }
}

/** @snipet [Play the test note when changing settings]*/
void test(){
  playNote(1000, 4 * 100, 0);
}


/** @brief Procedure that plays the selected lullaby
 *   
 *  @details These function play the lullaby that has been selected
 *					 The Lullaby indexes are passed onto the procedure via variable i
 *           The variable settings.deviceSettings[8] is used to determine whether the LED should pulse
 *           simultaneously as the lullaby plays.
 * 					 
 */
/** @snipet [Play lullaby]*/
void lullabySong(uint8_t i){
  bool led = false;
  if(settings.deviceSettings[8] == 2)led = true;                // led will pulse simulatenously as the lullaby plays
  switch(i){
    case 0:																											// first lullaby
      for (int i = 0; i < 4; i++) {
        if (lullaby0[i] == 0) {
          nrf_delay_ms(lullabyBeats0[i] * tempo); // rest
        } else {
          playNote(lullaby0[i], lullabyBeats0[i] * tempo,led);
        }
        // pause between notes
        nrf_delay_ms(tempo / 2); 
      }
      break;
    case 1:																											// second lullaby
      for (int i = 0; i < 5; i++) {
        if (lullaby1[i] == ' ') {
          nrf_delay_ms(lullabyBeats1[i] * tempo); // rest
        } else {
          playNote(lullaby1[i], lullabyBeats1[i] * tempo,led);
        }
        // pause between notes
        nrf_delay_ms(tempo / 2); 
      }
      break;
    case 2:																										 // third lullaby
      for (int i = 0; i < 18; i++) {
        if (lullaby2[i] == ' ') {
          nrf_delay_ms(lullabyBeats2[i] * tempo); // rest
        } else {
          playNote(lullaby2[i], lullabyBeats2[i] * tempo,led);
        }
        // pause between notes
        nrf_delay_ms(tempo / 2); 
      }
      break;
    case 3:																										// fourth lullaby
      for (int i = 0; i < 14; i++) {
        if (lullaby3[i] == ' ') {
          nrf_delay_ms(lullabyBeats3[i] * tempo); // rest
        } else {
          playNote(lullaby3[i], lullabyBeats3[i] * tempo,led);
        }
        // pause between notes
        nrf_delay_ms(tempo / 2); 
      }
      break;
  }
}

/** @brief Procedure that controls the lullaby playing event
 *   
 *  @details This function manages the different aspects of the lullaby playing event depending on the user settings.
 *					 It uses settings.deviceSettings[8] to determine when the LEDs should pulse
 *           It uses settings.deviceSettings[3] to determine the number of LED pulses
 *           settings.deviceSettings[7] determines the lullaby melody to play
 *           settings.deviceSettings[0] determines the volume of the melody
 *					 settings.deviceSettings[2] determines the number of times the melody will be played
 *           settings.deviceSettings[4] determines whether the first cycle will be louder than the rest
 * 					 
 */
/** @snipet [Control how lullaby plays]*/
void playLullaby(){
  if(settings.deviceSettings[8] == 2){ // play melody simulateneously
    if(settings.deviceSettings[3] > 0)NRF_TIMER2->TASKS_START = 1;  // enable timer interrupt
    uint8_t buff = 0;
    if(settings.deviceSettings[7] != 4 && settings.deviceSettings[0] > 0){
      for(int i = 0; i < settings.deviceSettings[2]; i++){
        // if 1st cycle is set to be louder 
        if(settings.deviceSettings[4] == 1){
          if(i == 0){
            buff = settings.deviceSettings[0]; // default volume
            settings.deviceSettings[0] = 100; // set maximum volume
          }
          else{
            settings.deviceSettings[0] = buff; // revert back to default volume
          }
        }
        lullabySong(settings.deviceSettings[7]);
        for(int i = 0; i < 1000; i++){
          analogWrite(BATLED1, fade);
					analogWrite(BATLED2, fade);
          nrf_delay_ms(1);
        }
      }
    }
    babyCrying = false;
    changePinStates(1);
  }
  else if(settings.deviceSettings[8] == 0){ // play melody before light pulses
    uint8_t buff = 0;
    if(settings.deviceSettings[7] != 4 && settings.deviceSettings[0] > 0){
      for(int i = 0; i < settings.deviceSettings[2]; i++){
        // if 1st cycle is set to be louder 
        if(settings.deviceSettings[4] == 1){
          if(i == 0){
            buff = settings.deviceSettings[0]; // default volume
            settings.deviceSettings[0] = 100; // set maximum volume
          }
          else{
            settings.deviceSettings[0] = buff; // revert back to default volume
          }
        }
        lullabySong(settings.deviceSettings[7]); // play lullaby
      }
    }
    // light pulses
    if(settings.deviceSettings[3] > 0){ 
      NRF_TIMER2->TASKS_START = 1;  // enable timer interrupt for led pulses
      nrf_delay_ms(10);
      for(int i = 0; i < 10000; i++){ // can go up to 50 pulses
        analogWrite(BATLED1, fade);
			  analogWrite(BATLED2, fade);
        if(fadeTrack == 0 && fade == 0) break;
        nrf_delay_ms(10);
      }
      nrf_gpio_pin_set(BATLED1);
			nrf_gpio_pin_set(BATLED2);
    }
    babyCrying = false;
    changePinStates(1);
  }
  else if(settings.deviceSettings[8] == 1){ // play melody after light pulses
    uint8_t buff = 0;
    // light pulses
    if(settings.deviceSettings[3] > 0){ 
      NRF_TIMER2->TASKS_START = 1;  // enable timer interrupt for led pulses
      nrf_delay_ms(10);
      for(int i = 0; i < 10000; i++){ // can go up to 50 pulses
        analogWrite(BATLED1, fade);
				analogWrite(BATLED2, fade);
        if(fadeTrack == 0 && fade == 0) break;
        nrf_delay_ms(10);
      }
      nrf_gpio_pin_set(BATLED1);
			nrf_gpio_pin_set(BATLED2);
    }
    //play lullaby
    if(settings.deviceSettings[7] != 4 && settings.deviceSettings[0] > 0){
      for(int i = 0; i < settings.deviceSettings[2]; i++){
        // if 1st cycle is set to be louder 
        if(settings.deviceSettings[4] == 1){
          if(i == 0){
            buff = settings.deviceSettings[0]; // default volume
            settings.deviceSettings[0] = 100; // set maximum volume
          }
          else{
            settings.deviceSettings[0] = buff; // revert back to default volume
          }
        }
        lullabySong(settings.deviceSettings[7]); // play lullaby
      }
    }
    babyCrying = false;
    changePinStates(1);
  }
}

/** @brief IRQ handler for timer2 event
 *   
 *  @details This function controls the LED pulses during the lullaby
 *					 I commented out the timer2_IRQhandler function from nrf_drv_timer.c file, to prevent the 'multiply defined' error
 * 					 
 */
/** @snipet [Control LED pulses]*/
void TIMER2_IRQHandler(void){
  if (NRF_TIMER2->EVENTS_COMPARE[0] != 0){
    // manage glowing and fading
    if (fade >= settings.deviceSettings[1]){
      tSign = -1;
      fadeTrack++;
    }
    else if (fade <= 0){
      tSign = 1;
      fadeTrack++;
    }
    fade += tSign;

    // detect when lullaby is over and continue LED fading
    if(babyCrying == false){
      analogWrite(BATLED1,fade);
			analogWrite(BATLED2,fade);
    }
    
    // detect when maximum number of fades is reached and end timer interrupt
    if(fadeTrack == settings.deviceSettings[3] * 2 && fade == 0){
      fadeTrack = 0;                     // initialize number of fades
      fade = 0;                          // initialize fade value
      nrf_gpio_pin_set(BATLED1);         // completely turn off LED
			nrf_gpio_pin_set(BATLED2);         // completely turn off LED
			NRF_TIMER2->EVENTS_COMPARE[0] = 0; // cleare the compare flag
      NRF_TIMER2->TASKS_STOP = 1;        // disable timer interrupt
    }
    NRF_TIMER2->EVENTS_COMPARE[0] = 0;
  }
}

/*************************************************BLE communication functions**************************************************/
/**@brief Function for converting a character array into int
 *
 * @details This function will be called in the extractdata function to get integers from received string
 *          The startIndex marks the cell of the array where the conversion will start and the lastIndex 
 *					marks where the conversion will end
 */
static int charArrayToInt(char cRef[], uint8_t startIndex, uint8_t lastIndex){
	int x = 0;                                                         // holds the integer result after conversion
	uint8_t j = 0;                                                     // holds position of new line character
	
	for(uint8_t i = lastIndex; i >= startIndex; i--){                  // find position of new line character
		if(cRef[i] == '\n') j = i - 1;
	}
	printf("%d \n", j);
	for(uint8_t i = j; i >= startIndex; i--){                          // convert characters, between start index and new line character, to integers
		 x += ((cRef[i] - '0') * pow(10,(j - i)));
	}
	return x;
}

/**@brief Function for retrieving settings from received string
 *
 * @details This function will be called in the nus_data_handler and will extract the settings from the received string
 */
static char arrayNames[10] = {'V', 'L', 'N', 'P', 'S', 'W', 'A', 'M', 'B', 'O'};
void extractData(char c[]){
  if(c[0] == 'V'){
    settings.deviceSettings[0] = charArrayToInt(c, 1, 4);
    if(settings.deviceSettings[0] > 0){
      //printf("play test tone, %d", settings.deviceSettings[0]);
			/*changePinStates(0);
      test();
      changePinStates(1);*/
    }
  }
  else if(c[0] == 'L'){
    settings.deviceSettings[1] = charArrayToInt(c, 1, 4);
    //analogWrite(BATLED, settings.deviceSettings[1]); // display LED brightness
    //ledPulseTime = (255 * 4)/settings.deviceSettings[1]; // calculate time interval for fade
    //Serial.print("timer setting");Serial.println(ledPulseTime);
    //timer1Setup(ledPulseTime); // setup the LED pulse timer to correspond to the brightness.
    //delay(1000);
  }
  else if(c[0] == 'N')settings.deviceSettings[2] = charArrayToInt(c, 1, 4);
  else if(c[0] == 'P')settings.deviceSettings[3] = charArrayToInt(c, 1, 4);
  else if(c[0] == 'S')settings.deviceSettings[4] = charArrayToInt(c, 1, 4);
  else if(c[0] == 'W')settings.deviceSettings[5] = charArrayToInt(c, 1, 4);
  else if(c[0] == 'A')settings.deviceSettings[6] = charArrayToInt(c, 1, 4);
  else if(c[0] == 'M'){
    settings.deviceSettings[7] = charArrayToInt(c, 1, 4);
    if(settings.deviceSettings[0] > 0){
      //changePinStates(0);
      //lullabySong(settings.deviceSettings[7]);
      //changePinStates(1);
    }
  }
  else if(c[0] == 'B')settings.deviceSettings[8] = charArrayToInt(c, 1, 4);
  else if(c[0] == 'O')settings.deviceSettings[9] = charArrayToInt(c, 1, 4);
  else if(c[0] == '#'){
    //RFduinoBLE.sendFloat(batteryLevel()); // send battery voltage to phone app
  }
  else if(c[0] == '$'){
    for(uint8_t i = 0; i < 9; i++)settings.deviceSettings[i] = defaultSettings[i]; // return device back to default settings
  }
  else if(c[0] == '@'){
    //sendData();
  }
}

/***********************************************************************************************************************************************/

int main(void) {
	/*/getSavedValues();                                           // retrieve settings saved on falsh memory
	ledPulseTime = (100 * 4)/settings.deviceSettings[1];        // calculate time interval for fade
	
	uart_init();                            										// initialize UART
	RTC1Setup();																								// configure RTC1
	timer2Setup(ledPulseTime);                                  // configure timer2
	adcConfig();                            										// configure ADC
	
	// define pin configurations
	nrf_gpio_cfg_output(BLELED);
	nrf_gpio_cfg_output(BATLED1);
	nrf_gpio_cfg_output(BATLED2);
	nrf_gpio_cfg_input(piezoAnalog, NRF_GPIO_PIN_NOPULL);
	
	attachInterrupt(0, piezoINTPin, 1);                        // configure interrupt on pin 3
	PWM_config();                                              // configure pulse width modulation*/
	uart_init();                            										// initialize UART
	adcConfig();                            										// configure ADC
	nrf_adc_start();                        									 // start ADC conversion
	
  while (true) {
		//__SEV();
		//__WFE();
		//__WFE();
		/*if(cryDetected && fadeTrack == 0){
      printf(">>Baby crying\n");
			changePinStates(0);
      playLullaby();
    }
		else{
			knockDetect();
			if (taps != 5 && taps != 0 && (millis - timeTrack) > knockInterval && deviceState == OFF){ // accidental tap when device is in sleep mode
        printf(">>Accidental tap->shutdown\n");
        printf("%d\n",taps);
				taps = 0; // initialize taps
        nrf_delay_ms(1000);
        sendToSleep(); // send device to sleep
      }
      else if (taps != 5 && taps != 0 &&  deviceState == ON && (millis - timeTrack) > knockInterval){ // accidental tap when device is ON
        printf(">>Accidental tap\n");
        printf("%d\n",taps);
        taps = 0; // initialize taps
      }
      if(taps == 0 && deviceState == ON && !BLEConnected && !BLEWindow){
        //printf("detecting crying");
				isBabyCrying();
				printf("%f\n", analogSample);
				//printf("%d\n", fadeTrack);
				printf("%d\n", cryDetected);
      }
		}*/
		isBabyCrying();
		//printf("%f\n", analogSample);
		//printf("%d\n", fadeTrack);
		printf("%d\n", cryDetected);
		//nrf_delay_ms(10);
  }
}

#endif /* NRF51 */

/** @} */
/**@brief Function for the Power manager.
 *
static void power_manage(void){
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}*/
