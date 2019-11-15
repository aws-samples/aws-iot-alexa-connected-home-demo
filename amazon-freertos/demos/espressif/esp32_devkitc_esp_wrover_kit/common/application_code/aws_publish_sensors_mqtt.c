/*
 * Amazon FreeRTOS MQTT Echo Demo V1.2.6
 * Copyright (C) 2017 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://aws.amazon.com/freertos
 * http://www.FreeRTOS.org
 */


/**
 * @file aws_publish_sensors_mqtt.c
 * @brief A simple BME280 (temperature, humidity, barrometric preassure) and
 * BH1730 (luminocity) sensors MQTT publishing example example.
 *
 * It creates an MQTT client that sends sensors data to AWS IoT
 * Another task reads sensor values and store them in gloval variables
 * - "static" declared variables are reside in global RAM section vs local
 * variables that stays in the task stack memory that is dynamically allocated
 * within a heap space.
 *
 */

/* Standard includes. */
#include "string.h"
#include "stdio.h"
#include <driver/i2c.h>
#include <driver/gpio.h>
#include "bh1730.h"
#include "bme280.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_spi_flash.h"
#include "led_strip/led_strip.h"

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "message_buffer.h"
#include "queue.h"

/* MQTT includes. */
#include "aws_shadow.h"
#include "aws_mqtt_agent.h"

/* Credentials includes. */
#include "aws_clientcredential.h"

/* Json-related includes */
#include "json.h"

/* Demo includes. */
#include "aws_demo_config.h"
#include "aws_hello_world.h"

#define APP_TAG    "APP_MAIN"

#define DEVKIT_C_BOARD
/*#define PICO_BOARD */

#ifdef DEVKIT_C_BOARD
#pragma message ( "Selecting Devkit-C board" )
#define GPIO_LED     21
#define GPIO_BTN1    36
#define GPIO_BTN2    39
#define GPIO_BTN3    34
#define GPIO_BTN4    35
#define GPIO_SDA     22
#define GPIO_SCL     23
#else
#ifdef PICO_BOARD
#warning "Selectig Pico board"
#define GPIO_LED     23
#define GPIO_BTN1    36
#define GPIO_BTN2    37
#define GPIO_BTN3    38
#define GPIO_BTN4    39
#define GPIO_SDA     22
#define GPIO_SCL     21
#else
#error "No Board selected"
#endif
#endif /* ifdef DEVKIT_C_BOARD */

/** @brief
 *  Number of buttons on the board
 */
#define BUTTONS_NUMBER    4


/** @brief
 * Timeout for connection to the MQTT broker (150 milliseconds)
 */
#define MQTT_TIMEOUT    pdMS_TO_TICKS( 150 )


/** @brief
 * Time interval allocated for a button to stabilize its state after pressing or releasing
 */
#define BUTTON_DEBOUNECE_TIME_MS    25

/**
 * @brief MQTT client ID.
 *
 * It must be unique per MQTT broker.
 */
#define echoCLIENT_ID               ( ( const uint8_t * ) "esp32" )

/**
 * @brief The topic that the MQTT client both subscribes and publishes to.
 */
#define sensorTOPIC_NAME            ( ( const uint8_t * ) "freertos/demos/sensors" )
#define sensorBUTTON_TOPIC_NAME     ( ( const uint8_t * ) "freertos/demos/buttons" )


/**
 * @brief Dimension of the character array buffers used to hold data (strings in
 * this case) that is published to and received from the MQTT broker (in the cloud).
 */
#define echoMAX_DATA_LENGTH              256

/**
 * @brief A block time of 0 simply means "don't block".
 */
#define echoDONT_BLOCK                   ( ( TickType_t ) 0 )

/**
 * @brief Size of the message queue
 */
#define echoSENSORS_BUTTON_QUEUE_SIZE    5

/**
 * @brief GPIO to which the LED strip is attached
 */
#define LEDS_GPIO                        21

/**
 * @brief Number of LEDs in the strip
 */
#define LED_STRIP_LENGTH                 5U

/**
 * @brief Interrupt number for LED strip control
 */
#define LED_STRIP_RMT_INTR_NUM           19


/*-----------------------------------------------------------*/

/**
 * @brief Enum describing the button state
 */
typedef enum
{
    RELEASED,
    PRESSED
} ButtonState_t;

/**
 * @brief Enum for the button number
 */
typedef enum
{
    BUTTON_1 = 0,
    BUTTON_2,
    BUTTON_3,
    BUTTON_4
} ButtonNumber_t;

/**
 * @brief Enum describing message type which publishing task will receive
 */
typedef enum
{
    SENSORS_DATA,
    BUTTONS_DATA,
    LED_DATA
} MessageType_t;

/**
 * @brief Structure containing sensors data
 */
typedef struct
{
    int32_t lTemperature;
    uint32_t ulPressure;
    float fIlluminance;
    int32_t lHumidity;
} SensorsMessage_t;

/**
 * @brief Structure containing button state
 */
typedef struct
{
    ButtonNumber_t xButtonNumber;
    ButtonState_t xButtonState;
} ButtonMessage_t;


/**
  * @brief Structure describing the state a LED
  */
typedef struct {
    uint8_t ucRed;
    uint8_t ucGreen;
    uint8_t ucBlue;
} LedState_t;

/**
  @brief Structure describing the state of the LED strip
  */
typedef struct {
    LedState_t xState[LED_STRIP_LENGTH];
} LedStripState_t;
/**
 * @brief Datatype for sending data to the publishing task
 */
typedef struct
{
    union
    {
        SensorsMessage_t xSensorsMessage;
        ButtonMessage_t xButtonMessage;
        LedStripState_t xLedMessage;
    } xMessageContent;
    MessageType_t xMessageType;
} Message_t;

/*-----------------------------------------------------------*/

/**
 * @brief Implements the task that connects to and then publishes messages to the
 * MQTT broker.
 *
 * Messages are published every five seconds for a minute.
 *
 * @param[in] pvParameters Parameters passed while creating the task. Unused in our
 * case.
 */
static void prvMQTTConnectAndPublishTask( void * pvParameters );

/**
 * @brief Creates an MQTT client and then connects to the MQTT broker.
 *
 * The MQTT broker end point is set by clientcredentialMQTT_BROKER_ENDPOINT.
 *
 * @return pdPASS if everything is successful, pdFAIL otherwise.
 */
static BaseType_t prvCreateClientAndConnectToBroker( void );

/*-----------------------------------------------------------*/

/**
 * @brief The handle of the Shadow client object
 */
static ShadowClientHandle_t xShadowHandle = NULL;

/**
 * @brief The queue for data transmission from sensors task and buttons interrupts to sending task
 */
static QueueHandle_t prvSensorsDataQueueHandle = 0;

/**
 * @brief Times used for the button debouncing purposes
 */
static TickType_t pxDebonceTimes[ BUTTONS_NUMBER ] = {};

/**
 * @brief Mapping from button numbers to button pins
 */
static const uint8_t pucButtonsPinMapping[ BUTTONS_NUMBER ] = { GPIO_BTN1, GPIO_BTN2, GPIO_BTN3, GPIO_BTN4 };


/**
 * @brief Json template for sending data about buttons state to the colud
 */
static const char pcButtonUpdateTemplate[] = "{"
                                             "\"state\":{"
                                             "\"reported\":{"
                                             "\"buttons\": {"
                                             "\"button%d\": %d"
                                             "}"
                                             "}"
                                             "},"
                                             "\"clentToken\": \"token-%d\""
                                             "}";

/**
 * @brief Json template for sending data about sensors state to the colud
 */
static const char pcSensorsUpdateTemplate[] = "{"
                                              "\"state\":{"
                                              "\"reported\":{"
                                              "\"sensors\":{"
                                              "\"temperature\": %f,"
                                              "\"pressure\": %f,"
                                              "\"humidity\": %f,"
                                              "\"illuminance\": %f"
                                              "}"
                                              "}"
                                              "},"
                                              "\"clentToken\": \"token-%d\""
                                              "}";

/**
 * @brief Json template for sending data about leds state to the colud
 */

#define LED_TEMPLATE \
    "{"              \
    "\"red\":%d,"    \
    "\"green\":%d,"  \
    "\"blue\":%d"    \
    "}"

static const char pcLEDUpdateTemplate[] = "{"
                                          "\"state\":{"
                                          "\"reported\":{"
                                          "\"leds\":["
                                          LED_TEMPLATE ","
                                          LED_TEMPLATE ","
                                          LED_TEMPLATE ","
                                          LED_TEMPLATE ","
                                          LED_TEMPLATE
                                          "]"
                                          "}"
                                          "},"
                                          "\"clentToken\": \"token-%d\""
                                          "}";



/**
 * @brief Buffers for containing states of LED strips
 */
static struct led_color_t led_strip_buf[ 2 ][ LED_STRIP_LENGTH ];

struct led_strip_t xLedStrip =
{
    .rgb_led_type      = RGB_LED_TYPE_WS2812,
    .rmt_channel       = RMT_CHANNEL_1,
    .rmt_interrupt_num = LED_STRIP_RMT_INTR_NUM,
    .gpio              = LEDS_GPIO,
    .led_strip_buf_1   = led_strip_buf[ 0 ],
    .led_strip_buf_2   = led_strip_buf[ 1 ],
    .led_strip_length  = LED_STRIP_LENGTH
};


/*-----------------------------------------------------------*/
static esp_err_t i2c_init()
{
    /*Initialize I2C port in master mode */
    esp_err_t ret;
    i2c_config_t conf;

    memset( &conf, 0, sizeof( conf ) );
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = GPIO_SDA;
    conf.scl_io_num = GPIO_SCL;
    conf.master.clk_speed = 100 * 1000; /*standard I2C speed */
    ret = i2c_param_config( I2C_NUM_1, &conf );

    if( ret != ESP_OK )
    {
        ESP_LOGE( APP_TAG, "I2C param config failed" );
        return ESP_FAIL;
    }

    ret = i2c_driver_install( I2C_NUM_1, I2C_MODE_MASTER, 0, 0, 0 );

    if( ret != ESP_OK )
    {
        ESP_LOGE( APP_TAG, "unable to install i2c driver" );
        return ESP_FAIL;
    }

    return ESP_OK;
}

/*-----------------------------------------------------------*/

static BaseType_t xLedInit()
{
    xLedStrip.access_semaphore = xSemaphoreCreateBinary();
    return led_strip_init( &xLedStrip ) ? pdTRUE : pdFALSE;
}

/*-----------------------------------------------------------*/

static void xLedSetColors( LedStripState_t * pxState)
{
    for (int i = 0; i < LED_STRIP_LENGTH; ++ i)
    {
        led_strip_set_pixel_rgb( &xLedStrip, i, pxState->xState[i].ucRed, pxState->xState[i].ucGreen, pxState->xState[i].ucBlue);
    }
    led_strip_show(&xLedStrip);

}

/*-----------------------------------------------------------*/
/*                 Buttons interrupt handlers                */

/* Interrupt handler for button pressing events */
void IRAM_ATTR handleButtonInterrupt( void * args )
{
    Message_t xMessage = {};
    BaseType_t bHigherPriorityTaskAwoken = pdFALSE;

    /* We pass the button number directly (not as pointer) as pointer type can store any int */
    ButtonNumber_t xButtonNumber = ( ButtonNumber_t ) args;

    /* We ignore all the interrupts after the first signal for the debouncing purposes.
     *  Initially pxDebonceTimes[xButtonNumber] contains 0, but we don't want to handle button events until the board is connected to
     *  the network anyway and it takes way longer than any conceivable BUTTON_DEBOUNECE_TIME_MS */
    if( xTaskGetTickCountFromISR() - pxDebonceTimes[ xButtonNumber ] <= pdMS_TO_TICKS( BUTTON_DEBOUNECE_TIME_MS ) )
    {
        return;
    }

    /* Disable interrupts for the duration of processing */
    BaseType_t uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();

    /* The GPIO logic is inverted */
    ButtonState_t xCurrentState = gpio_get_level( pucButtonsPinMapping[ ( int ) xButtonNumber ] ) ? RELEASED : PRESSED;

    xMessage.xMessageType = BUTTONS_DATA;
    xMessage.xMessageContent.xButtonMessage.xButtonNumber = xButtonNumber;
    xMessage.xMessageContent.xButtonMessage.xButtonState = xCurrentState;
    xQueueSendFromISR( prvSensorsDataQueueHandle, &xMessage, &bHigherPriorityTaskAwoken );
    pxDebonceTimes[ xButtonNumber ] = xTaskGetTickCountFromISR();
    taskEXIT_CRITICAL_FROM_ISR( uxSavedInterruptStatus );

    /* Now the buffer is empty we can switch context if necessary. */
    if( bHigherPriorityTaskAwoken )
    {
        /* Actual macro used here is port specific. */
        portYIELD_FROM_ISR();
    }
}

/*-----------------------------------------------------------*/
static esp_err_t gpio_buttons_init()
{
    /* Initialize ISR server for setting custom handlers for each GPIO */
    gpio_install_isr_service( ESP_INTR_FLAG_EDGE );
    esp_err_t xStatus = ESP_OK;

    if( xStatus == ESP_OK )
    {
        /* Initialize the GPIO pin */
        gpio_config_t xGPIOConfig = {};
        xGPIOConfig.intr_type = GPIO_INTR_ANYEDGE;
        xGPIOConfig.mode = GPIO_MODE_INPUT;
        xGPIOConfig.pull_down_en = 1;
        xGPIOConfig.pull_up_en = 0;

        for( int i = 0; i < BUTTONS_NUMBER; ++i )
        {
            xGPIOConfig.pin_bit_mask |= 1ull << pucButtonsPinMapping[ i ];
        }

        xStatus = gpio_config( &xGPIOConfig );
    }

    for( int i = 0; i < BUTTONS_NUMBER; ++i )
    {
        if( xStatus == ESP_OK )
        {
            /* Set interrupt types for buttons */
            xStatus = gpio_set_intr_type( pucButtonsPinMapping[ i ], GPIO_INTR_ANYEDGE );
        }

        if( xStatus == ESP_OK )
        {
            /* Set interrupt types for buttons */
            xStatus = gpio_set_intr_type( pucButtonsPinMapping[ i ], GPIO_INTR_ANYEDGE );
        }

        if( xStatus == ESP_OK )
        {
            /* Set interrupt handler for buttons */
            xStatus = gpio_isr_handler_add( pucButtonsPinMapping[ i ], handleButtonInterrupt, ( void * ) i );
        }

        if( xStatus == ESP_OK )
        {
            /* Enable the interrupt */
            xStatus = gpio_intr_enable( pucButtonsPinMapping[ i ] );
        }
    }

    return xStatus;
}
/*-----------------------------------------------------------*/

static json_object_entry * find_key( json_value * o,
                                     const char * key )
{
    if( o == NULL )
    {
        return NULL;
    }

    for( int i = 0; i < o->u.object.length; ++i )
    {
        if( strcmp( o->u.object.values[ i ].name, key ) == 0 )
        {
            return &o->u.object.values[ i ];
        }
    }

    configPRINTF( ( "Json parse: Key not found: %s\n", key ) );
    return NULL;
}

/*-----------------------------------------------------------*/

static BaseType_t prvDeltaCallback( void * pvUserData,
                                    const char * const pcThingName,
                                    const char * const pcDeltaDocument,
                                    uint32_t ulDocumentLength,
                                    MQTTBufferHandle_t xBuffer )
{
    ( void ) xBuffer;
    ( void ) pvUserData;
    ( void ) pcThingName;
    configPRINTF( ( "Received a delta document: %s\r\n", pcDeltaDocument ) );
    json_value * xJsonDoc = json_parse( pcDeltaDocument, ulDocumentLength );
    json_object_entry * pxJsonState = find_key( xJsonDoc, "state" );
    if (!xJsonDoc)
    {
        json_value_free( xJsonDoc );
        SHADOW_ReturnMQTTBuffer( xShadowHandle, xBuffer );
        return pdTRUE;
    }
    json_object_entry * pxJsonLedsArray = find_key( pxJsonState->value, "leds" );
    LedStripState_t xLedStripState = {};
    Message_t xMessage = {};
    if( pxJsonLedsArray )
    {
        for( int i = 0; i < pxJsonLedsArray->value->u.array.length; ++i )
        {
            json_value * xLedColorValue = pxJsonLedsArray->value->u.array.values[ i ];
            json_object_entry * xRedValue = find_key( xLedColorValue, "red" );
            json_object_entry * xGreenValue = find_key( xLedColorValue, "green" );
            json_object_entry * xBlueValue = find_key( xLedColorValue, "blue" );
            /* Check if all three colors are present in the dictionary */
            if( xBlueValue && xGreenValue && xRedValue )
            {
                xLedStripState.xState[i].ucRed = xRedValue->value->u.integer;
                xLedStripState.xState[i].ucGreen = xGreenValue->value->u.integer;
                xLedStripState.xState[i].ucBlue = xBlueValue->value->u.integer;
            }
        }
        xLedSetColors( &xLedStripState );
        xMessage.xMessageType = LED_DATA;
        xMessage.xMessageContent.xLedMessage = xLedStripState;
        ( void ) xQueueSend( prvSensorsDataQueueHandle, &xMessage, portMAX_DELAY );
    }

    json_value_free( xJsonDoc );
    SHADOW_ReturnMQTTBuffer( xShadowHandle, xBuffer );
    return pdTRUE;
}



static ShadowReturnCode_t prvGetState()
{
    ShadowOperationParams_t xOperationParams;
    ShadowReturnCode_t xReturn;

    memset( &xOperationParams, 0, sizeof( xOperationParams ) );
    xOperationParams.pcThingName = clientcredentialIOT_THING_NAME;
    xOperationParams.xQoS = 1;
    printf( "Shadow Get \n" );
    xReturn = SHADOW_Get( xShadowHandle, &xOperationParams, MQTT_TIMEOUT * 3 );
    printf( "Shadow Get return: %d\n", xReturn );

    if( xReturn != eShadowSuccess )
    {
        return xReturn;
    }
    json_value * val = json_parse( xOperationParams.pcData, xOperationParams.ulDataLength );
    json_object_entry * state = find_key( val, "state" );

    if( !state )
    {
        json_value_free( val );
        SHADOW_ReturnMQTTBuffer( xShadowHandle, xOperationParams.xBuffer );
        return pdTRUE;
    }

    json_object_entry * reported = find_key( state->value, "reported" );

    if( !reported )
    {
        json_value_free( val );
        return pdTRUE;
    }

    json_object_entry * pxJsonLedsArray = find_key( reported->value, "leds" );
    LedStripState_t xLedStripState = {};
    Message_t xMessage = {};
    if( pxJsonLedsArray )
    {
        for( int i = 0; i < pxJsonLedsArray->value->u.array.length; ++i )
        {
            json_value * xLedColorValue = pxJsonLedsArray->value->u.array.values[ i ];
            json_object_entry * xRedValue = find_key( xLedColorValue, "red" );
            json_object_entry * xGreenValue = find_key( xLedColorValue, "green" );
            json_object_entry * xBlueValue = find_key( xLedColorValue, "blue" );
            /* Check if all three colors are present in the dictionary */
            if( xBlueValue && xGreenValue && xRedValue )
            {
                xLedStripState.xState[i].ucRed = xRedValue->value->u.integer;
                xLedStripState.xState[i].ucGreen = xGreenValue->value->u.integer;
                xLedStripState.xState[i].ucBlue = xBlueValue->value->u.integer;
            }
        }
        xLedSetColors( &xLedStripState );
    }
    json_value_free( val );
    SHADOW_ReturnMQTTBuffer( xShadowHandle, xOperationParams.xBuffer );
    return xReturn;
}

/*-----------------------------------------------------------*/

static BaseType_t prvCreateClientAndConnectToBroker( void )
{
    ShadowReturnCode_t xReturned;
    BaseType_t xReturn = pdFAIL;
    ShadowCreateParams_t xCreateParam;
    ShadowCallbackParams_t xCallbackParams;

    xCreateParam.xMQTTClientType = eDedicatedMQTTClient;

    MQTTAgentConnectParams_t xConnectParameters =
    {
        clientcredentialMQTT_BROKER_ENDPOINT,               /* The URL of the MQTT broker to connect to. */
        democonfigMQTT_AGENT_CONNECT_FLAGS,                 /* Connection flags. */
        pdFALSE,                                            /* Deprecated. */
        clientcredentialMQTT_BROKER_PORT,                   /* Port number on which the MQTT broker is listening. Can be overridden by ALPN connection flag. */
        ( const uint8_t * ) clientcredentialIOT_THING_NAME, /* Client Identifier of the MQTT client. It should be unique per broker. */
        0,                                                  /* The length of the client Id, filled in later as not const. */
        pdFALSE,                                            /* Deprecated. */
        &xShadowHandle,                                     /* User data supplied to the callback. Can be NULL. */
        NULL,                                               /* Callback used to report various events. Can be NULL. */
        NULL,                                               /* Certificate used for secure connection. Can be NULL. */
        0                                                   /* Size of certificate used for secure connection. */
    };

    /* Check this function has not already been executed. */
    configASSERT( xShadowHandle == NULL );

    /* The shadow client object must be created before it can be used. */
    xReturned = SHADOW_ClientCreate( &xShadowHandle, &xCreateParam );

    if( xReturned == eShadowSuccess )
    {
        /* Fill in the MQTTAgentConnectParams_t member that is not const,
         * and therefore could not be set in the initializer (where
         * xConnectParameters is declared in this function). */
        xConnectParameters.usClientIdLength = ( uint16_t ) strlen( ( const char * ) clientcredentialIOT_THING_NAME );

        /* Connect to the broker. */
        configPRINTF( ( "MQTT  attempting to connect to %s.\r\n", clientcredentialMQTT_BROKER_ENDPOINT ) );
        xReturned = SHADOW_ClientConnect( xShadowHandle,
                                          &xConnectParameters,
                                          democonfigMQTT_ECHO_TLS_NEGOTIATION_TIMEOUT );
    }

    if( xReturned == eShadowSuccess )
    {
        xCallbackParams.pcThingName = clientcredentialIOT_THING_NAME;
        xCallbackParams.xShadowDeletedCallback = NULL;
        xCallbackParams.xShadowDeltaCallback = prvDeltaCallback;
        xCallbackParams.xShadowUpdatedCallback = NULL;
        xReturned = SHADOW_RegisterCallbacks( xShadowHandle, &xCallbackParams, MQTT_TIMEOUT * 3 );
    }

    if( xReturned != eShadowSuccess )
    {
        /* Could not connect, so delete the MQTT client. */
        ( void ) SHADOW_ClientDelete( xShadowHandle );
        configPRINTF( ( "ERROR:  MQTT echo failed to connect.\r\n" ) );
    }
    else
    {
        configPRINTF( ( "MQTT echo connected.\r\n" ) );
        xReturn = pdPASS;
    }

    return xReturn;
}

/*-----------------------------------------------------------*/

void prvGetSensorValueTask( void * pvParams )
{
    ( void ) pvParams;

    configPRINTF( ( "Enter sensors task \r\n" ) );
    Message_t xMessage = {};
    ESP_LOGI( APP_TAG, "Initializing light sensor" );
    bh1730_t * bh = bh1730_init( I2C_NUM_1, 0x29 );
    /* Current tick to use in vTaskDelayUntil */
    TickType_t xCurrentTick = xTaskGetTickCount();

    if( bh == NULL )
    {
        ESP_LOGE( APP_TAG, "Could not initialize light sensor" );
        vTaskDelete( NULL );
    }

    ESP_LOGI( APP_TAG, "Light sensor Initialized" );
    vTaskDelay( 1667 / portTICK_RATE_MS );

    ESP_LOGI( APP_TAG, "Initializing temperature sensor" );
    bme280_t * bme = bme280_init( I2C_NUM_1, 0x76 );

    if( bme == NULL )
    {
        ESP_LOGE( APP_TAG, "Could not initialize temperature sensor" );
        vTaskDelete( NULL );
    }

    ESP_LOGI( APP_TAG, "Temperature sensor Initialized" );
    vTaskDelay( 1667 / portTICK_RATE_MS );

    while( 1 )
    {
        xMessage.xMessageContent.xSensorsMessage.fIlluminance = bh1730_read_lux( bh );
        bme280_read( bme, &xMessage.xMessageContent.xSensorsMessage.lTemperature,
                     &xMessage.xMessageContent.xSensorsMessage.ulPressure,
                     &xMessage.xMessageContent.xSensorsMessage.lHumidity
                     );
        xMessage.xMessageType = SENSORS_DATA;

        /* If queue is full for the full cycle consider the read data obsolete and read them again.
         * In this case both results of sending to the queue can be considered correct
         * so it is not required to check the return value.
         */
        ( void ) xQueueSend( prvSensorsDataQueueHandle, &xMessage, pdMS_TO_TICKS( 3000 ) );
        vTaskDelayUntil( &xCurrentTick, pdMS_TO_TICKS( 5000 ) );
    }
}



/*-----------------------------------------------------------*/

static void prvMQTTConnectAndPublishTask( void * pvParameters )
{
    BaseType_t xMQTTMessageNumber = 0, xReturned = pdPASS;
    TaskHandle_t xGetSensorValueTask = NULL;
    Message_t xMessage = {};
    char cDataBuffer[ echoMAX_DATA_LENGTH ];
    ShadowOperationParams_t xUpdateParam;
    LedState_t * pxLedState;

    /* Avoid compiler warnings about unused parameters. */
    ( void ) pvParameters;
    configPRINTF( ( "Enter prvMQTTConnectAndPublishTask \r\n" ) );
    vTaskDelay( 1 );
    /* Create the MQTT client object and connect it to the MQTT broker. */
    xReturned = prvCreateClientAndConnectToBroker();
    if( xReturned == pdPASS )
    {
        vTaskDelay( 1 );
        prvGetState();
    }

    if( xReturned == pdPASS )
    {
        /* Create the task that generates random number */
        xReturned = xTaskCreate( prvGetSensorValueTask,               /* The function that implements the task. */
                                 "ReadSensorsValues",                 /* Human readable name for the task. */
                                 democonfigMQTT_ECHO_TASK_STACK_SIZE, /* Size of the stack to allocate for the task, in words not bytes! */
                                 NULL,                                /* The task parameter is not used. */
                                 tskIDLE_PRIORITY,                    /* Runs at the lowest priority. */
                                 &( xGetSensorValueTask ) );          /* The handle is stored so the created task can be deleted again at the end of the demo. */

        if( xReturned != pdPASS )
        {
            /* The task could not be created because there was insufficient FreeRTOS
             * heap available to create the task's data structures and/or stack. */
            configPRINTF( ( "Sensors gathering task could not be created - out of heap space?\r\n" ) );
        }
    }
    else
    {
        configPRINTF( ( "Cannot connect to MQTT broker.\r\n" ) );
    }

    if( xReturned == pdPASS )
    {
        /* MQTT client is now connected to a broker. Wait for messages and publish them. */
        for( ; ; )
        {
            /* We wait for the message indefinitely, so we don't need to check for the return code */
            ( void ) xQueueReceive( prvSensorsDataQueueHandle, &xMessage, portMAX_DELAY );

            switch( xMessage.xMessageType )
            {
                case BUTTONS_DATA:
                    configPRINTF( ( "Get the button data: n: %d, s: %d \r\n", xMessage.xMessageContent.xButtonMessage.xButtonNumber, xMessage.xMessageContent.xButtonMessage.xButtonState ) );
                    ( void ) snprintf( cDataBuffer, echoMAX_DATA_LENGTH, pcButtonUpdateTemplate,
                                       xMessage.xMessageContent.xButtonMessage.xButtonNumber + 1,
                                       xMessage.xMessageContent.xButtonMessage.xButtonState,
                                       ( int ) xMQTTMessageNumber );
                    break;

                case SENSORS_DATA:
                    configPRINTF( ( "Get the sensors data: t: %d, p: %u, h: %u, i: %f \r\n",
                                    xMessage.xMessageContent.xSensorsMessage.lTemperature,
                                    xMessage.xMessageContent.xSensorsMessage.ulPressure,
                                    xMessage.xMessageContent.xSensorsMessage.lHumidity,
                                    xMessage.xMessageContent.xSensorsMessage.fIlluminance ) );

                    /* Create the message that will be published, which is of the form "Hello World n"
                     * where n is a monotonically increasing number. Note that snprintf appends
                     * terminating null character to the cDataBuffer. */
                    ( void ) snprintf( cDataBuffer, echoMAX_DATA_LENGTH, pcSensorsUpdateTemplate,
                                       xMessage.xMessageContent.xSensorsMessage.lTemperature / 100.,
                                       ( double ) xMessage.xMessageContent.xSensorsMessage.ulPressure,
                                       ( double ) xMessage.xMessageContent.xSensorsMessage.lHumidity,
                                       xMessage.xMessageContent.xSensorsMessage.fIlluminance,
                                       ( int ) xMQTTMessageNumber );

                    break;
                case LED_DATA:
                    pxLedState = xMessage.xMessageContent.xLedMessage.xState;
                    ( void ) snprintf( cDataBuffer, echoMAX_DATA_LENGTH, pcLEDUpdateTemplate,
                                       pxLedState[0].ucRed, pxLedState[0].ucGreen, pxLedState[0].ucBlue,
                                       pxLedState[1].ucRed, pxLedState[1].ucGreen, pxLedState[1].ucBlue,
                                       pxLedState[2].ucRed, pxLedState[2].ucGreen, pxLedState[2].ucBlue,
                                       pxLedState[3].ucRed, pxLedState[3].ucGreen, pxLedState[3].ucBlue,
                                       pxLedState[4].ucRed, pxLedState[4].ucGreen, pxLedState[4].ucBlue,
                                       ( int ) xMQTTMessageNumber );
                break;
                default:
                    break;
            }

            /* Fill the common update parameters */
            xUpdateParam.pcThingName = clientcredentialIOT_THING_NAME;
            xUpdateParam.xQoS = eMQTTQoS0;
            xUpdateParam.pcData = cDataBuffer;
            xUpdateParam.ucKeepSubscriptions = pdTRUE;
            xUpdateParam.ulDataLength = ( uint32_t ) strlen( cDataBuffer );
            /* Publish the message */
            configPRINTF( ( "%s \r\n", cDataBuffer ) );
            xReturned = SHADOW_Update( xShadowHandle, &xUpdateParam, MQTT_TIMEOUT * 3 );

            if( xReturned == eShadowSuccess )
            {
                configPRINTF( ( "Successfully performed update.\r\n" ) );
            }
            else
            {
                configPRINTF( ( "Update failed, returned %d.\r\n", xReturned ) );
            }

            xMQTTMessageNumber += 1;
        }
        /* Disconnect the client. */
        ( void ) SHADOW_ClientDisconnect( xShadowHandle );
    }



    /* End the demo by deleting all created resources. */
    configPRINTF( ( "MQTT echo demo finished.\r\n" ) );
    vTaskDelete( NULL ); /* Delete this task. */
}
/*-----------------------------------------------------------*/

void vStartSensorsDemo( void )
{
    configPRINTF( ( "Creating MQTT Echo Task...\r\n" ) );
    BaseType_t xStatus = pdPASS;

    /* Initialize I2C peripherals */
    esp_err_t xInitStatus = i2c_init();
    xStatus = ( xInitStatus == ESP_OK ) ? pdPASS : pdFAIL;

    if( xStatus == pdPASS )
    {
        xStatus = xLedInit();
    }

    /* Initialize buttons */
    if( xStatus == pdPASS )
    {
        xInitStatus = gpio_buttons_init();
        xStatus = ( xInitStatus == ESP_OK ) ? pdPASS : pdFAIL;
    }

    /* Initialize the queue for sensor data and button state messages */
    if( xStatus == pdPASS )
    {
        prvSensorsDataQueueHandle = xQueueCreate( echoSENSORS_BUTTON_QUEUE_SIZE, sizeof( Message_t ) );

        if( prvSensorsDataQueueHandle == NULL )
        {
            xStatus = pdFAIL;
        }
    }

    /* Create the task that publishes messages to the MQTT broker every five
     * seconds.  This task, in turn, creates the task that echoes data received
     * from the broker back to the broker. */
    if( xStatus == pdPASS )
    {
        xStatus = xTaskCreate( prvMQTTConnectAndPublishTask,        /* The function that implements the demo task. */
                               "MQTTEcho",                          /* The name to assign to the task being created. */
                               democonfigMQTT_ECHO_TASK_STACK_SIZE, /* The size, in WORDS (not bytes), of the stack to allocate for the task being created. */
                               NULL,                                /* The task parameter is not being used. */
                               democonfigMQTT_ECHO_TASK_PRIORITY,   /* The priority at which the task being created will run. */
                               NULL );                              /* Not storing the task's handle. */
    }

    if( xStatus == pdFAIL )
    {
        configPRINTF( ( "Cannot create MQTT Echo Task\r\n" ) );
    }
    else
    {
        configPRINTF( ( "MQTT Echo Task created\r\n" ) );
    }
}
/*-----------------------------------------------------------*/
