/* FreeRTOS.org includes. */
#include "FreeRTOS_AVR.h"
//#include "task.h"

/* Demo includes. */
#include "basic_io_avr.h"

/* Arduino Libs */
#include <Servo.h> 
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"

/* Sonar Lib */
#include <NewPing.h>


#define MOTEUR_DROIT_PIN    10      // Arduino pin tied to servo_sonar.
#define MOTEUR_GAUCHE_PIN   11      // Arduino pin tied to servo_sonar.

#define SERVO_SONAR_PIN     12      // Arduino pin tied to servo_sonar.

#define TX_GND_PIN          A0      // Arduino pin tied to emetteur GND.
#define TX_VCC_PIN          A1      // Arduino pin tied to emetteur VCC.
#define TX_DATA_PIN         A2      // Arduino pin tied to emetteur DATA.

#define IR_DROITE_PIN        2      // Arduino pin tied to IR droite.
#define IR_BAS_DROIT_PIN    3      // Arduino pin tied to IR gauche.
#define IR_BAS_GAUCHE_PIN    4      // Arduino pin tied to IR bas.
#define IR_GAUCHE_PIN        5      // Arduino pin tied to IR avant.

#define TRIGGER_PIN          6      // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN             7      // Arduino pin tied to echo pin on the ultrasonic sensor.




#define TRIGGER_PIN          6      // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN             7      // Arduino pin tied to echo pin on the ultrasonic sensor
#define MAX_DISTANCE         200    // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

#define LPF_param           .4



/* The task function. */
void vTaskServo( void *pvParameters );
void vTaskMeasure( void *pvParameters );
void vTaskCommand( void *pvParameters );

/* functions */
void avance(int vitesse_moteur);
void tourne(int angle, int vitesse_moteur);

/* Inter-process Communication */
SemaphoreHandle_t sem_new_angle;
SemaphoreHandle_t mutex_tab;

/* Shared Ressources */
int current_angle;
char RIGHT_OBSTACLE,LEFT_OBSTACLE,FRONT_OBSTACLE;


/* Define the strings that will be passed in as the task parameters.  These are
   defined const and off the stack to ensure they remain valid when the tasks are
   executing. */
const char *pcTextForTaskServo = "Task Servo is running\r\n";
const char *pcTextForTaskMeasure = "Task Measure is running\r\n";
const char *pcTextForTaskCommand = "Task Command is running\r\n";


/*-----------------------------------------------------------*/

void setup( void )
{
    // Insure malloc works in tasks
    __malloc_heap_end = (char*)RAMEND;

    Wire.begin();

    Serial.begin(115200);


    /* Create the first task at priority 3 */
    xTaskCreate( vTaskServo, "Task Servo -", 200, (void*)pcTextForTaskServo, 0, NULL );

    /* ... and the second task at priority 2.*/
    xTaskCreate( vTaskMeasure, "Task Measure", 200, (void*)pcTextForTaskMeasure, 0, NULL );


    xTaskCreate( vTaskCommand, "Task Command", 100, (void*)pcTextForTaskCommand, 0, NULL );

    /* Start the scheduler so our tasks start executing. */
    vTaskStartScheduler();

    Serial.println(F("Insufficient RAM"));
    for( ;; );
    //  return 0;
}
/*-----------------------------------------------------------*/


/* Angle Config */
#define ANGLE_MAX   160 // Travel of sonar servo
#define ANGLE_MIN   20 // Travel of sonar servo
#define SERVO_STEP   5 // Step of sonar servo in degree
#define FRONT_ANGLE 10 // Tolerance to determine obstacle orientation
#define OBS_MIN 100
void vTaskServo( void *pvParameters )
{
    /* Variable Definition */
    char *pcTaskName;
    TickType_t xLastWakeTime;
    char sens = 0; // Sens 0 : Positive, Sens 1 : Negative
    unsigned int distance,uS;

    /* Initialize current_angle variable */
    current_angle = 0;

    /* Prepare Servo Sonar */
    Servo servo_sonar;
    servo_sonar.attach(SERVO_SONAR_PIN);

    pcTaskName = ( char * ) pvParameters;

    /* Initialize variable with current tick count */
    xLastWakeTime = xTaskGetTickCount();


    /* Print out the name of this task. */
    vPrintString( pcTaskName );

    /* Initialize Sonar Sensor */
    NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);     // NewPing setup of pins and maximum distance.

    for( ;; )
    {
        if(sens == 0){
            if(current_angle < ANGLE_MAX) current_angle+=SERVO_STEP;
            else  sens = 1;
        }
        else{       // if sens == 1
            if(current_angle > ANGLE_MIN)  current_angle -=SERVO_STEP;
            else  sens = 0;
        }

        servo_sonar.write(current_angle);

        /* Wait 15 MS for servo to set angle */
        vTaskDelay((15L * configTICK_RATE_HZ) / 1000L);


        uS = sonar.ping(); // Send ping, get ping time in microseconds (uS).
        distance = uS/(0.1*US_ROUNDTRIP_CM);
        if(distance != 0){
        /* Check for obstacle and return direction */
        if(distance < OBS_MIN){
            if(current_angle < 90 - FRONT_ANGLE){
                vPrintString("Servo - Obstacle Right\r\n");
                RIGHT_OBSTACLE = 1;
            }
            else if(current_angle > 90 + FRONT_ANGLE){
                vPrintString("Servo - Obstacle Left\r\n");
                LEFT_OBSTACLE = 1;
            }
            else{
                //FRONT_OBSTACLE = 1;
                vPrintString("Servo - Obstacle Front\r\n");
            }
        }
        }
        vTaskDelayUntil( &xLastWakeTime, ( 40 / portTICK_PERIOD_MS ) );
    }
}
//------------------------------------------------------------------------------
#define RETURN_ACC 7000
#define LPF 0.1
void vTaskMeasure( void *pvParameters )
{
    /* Variable Definition */
    char *pcTaskName;
    TickType_t xLastWakeTime;
    MPU6050 mpu;
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    int16_t integral,lpf_ay;

    pcTaskName = ( char * ) pvParameters;

    /* Initialize IMU */
    vPrintString("Initialize MPU\r\n");
    mpu.initialize();
    vPrintString(mpu.testConnection() ? "Connected\r\n" : "Connection failed\r\n");

    /* Print out the name of this task. */
    vPrintString( pcTaskName );

    /* Initialize variable with current tick count */
    xLastWakeTime = xTaskGetTickCount();

    for( ;; )
    {
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        lpf_ay =(1-LPF)*lpf_ay + LPF*ay;
        //integral += 0.01*lpf_ay;
        Serial.println(lpf_ay);
        if(ay < (-RETURN_ACC)){
            FRONT_OBSTACLE = 1;
        }


        vTaskDelayUntil( &xLastWakeTime, ( 100 / portTICK_PERIOD_MS ) );

    }
}

/* Private ressources */
Servo moteur_droit;
Servo moteur_gauche;
//------------------------------------------------------------------------------
void vTaskCommand( void *pvParameters )
{
    /* Variable Definition */
    char *pcTaskName;
    TickType_t xLastWakeTime;

    /* Prepare Wheel Servo */
    moteur_droit.attach(MOTEUR_DROIT_PIN);
    moteur_gauche.attach(MOTEUR_GAUCHE_PIN);
    moteur_droit.attach(MOTEUR_DROIT_PIN);
    moteur_gauche.attach(MOTEUR_GAUCHE_PIN);

    pcTaskName = ( char * ) pvParameters;

    vPrintString( pcTaskName );

    xLastWakeTime = xTaskGetTickCount();
    int i;
    for( ;; )
    {
        if(RIGHT_OBSTACLE){
            //vPrintString("Command - Turn Left \r\n");
            tourne(-15,4);
            RIGHT_OBSTACLE = 0;
        }
        else if(LEFT_OBSTACLE){
            //vPrintString("Command - Turn Right \r\n");
            tourne(+15,4);
            LEFT_OBSTACLE = 0;
        }

        else if(FRONT_OBSTACLE){
            vPrintString("Command - Go Backward \r\n");
           avance(-4);
           FRONT_OBSTACLE = 0;
        }

        else {
            avance(4);
        }
        //avance(4);
        //moteur_droit.write(90);
        //moteur_gauche.write(90);
        /*RIGHT_OBSTACLE = 0;
        LEFT_OBSTACLE = 0;
        FRONT_OBSTACLE = 0;*/
    vTaskDelay(( 1000 / portTICK_PERIOD_MS ) );

    }
}


///////////AVANCE DROITE//////////////////////////////////////////////////////////////////////////////////////////////////////

void avance(int vitesse_moteur)
{
    moteur_droit.write(90 - vitesse_moteur*2.1);
    moteur_gauche.write(90 + vitesse_moteur);
}

///////////ROTATION SUR PLACE//////////////////////////////////////////////////////////////////////////////////////////////////

void tourne(int angle, int vitesse_moteur)
{
    int sens_rotation = angle/abs(angle);

    if( sens_rotation == 1)
    {
        moteur_droit.write((90 + vitesse_moteur*2.1) * sens_rotation) ;
        moteur_gauche.write((90 + vitesse_moteur) * sens_rotation) ;
        vTaskDelay((abs(angle*9.5))/ portTICK_PERIOD_MS);
    }
    if( sens_rotation == -1)
    {
        moteur_droit.write((90 + vitesse_moteur*2.1) * sens_rotation) ;
        moteur_gauche.write((90 + vitesse_moteur) * sens_rotation) ;
        vTaskDelay((abs(angle*6.5))/ portTICK_PERIOD_MS);
    }

    //Serial.print("sens rotation  = ") ; Serial.println(sens_rotation) ;
    moteur_droit.write(90);
    moteur_gauche.write(90);

}


//------------------------------------------------------------------------------
void loop() {}


