/* FreeRTOS.org includes. */
#include "FreeRTOS_AVR.h"
//#include "task.h"

/* Demo includes. */
#include "basic_io_avr.h"

/* Arduino Libs */
#include <Servo.h>
#include <Wire.h>
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


/* Servos */
Servo servo_sonar;
Servo moteur_droit;
Servo moteur_gauche;

/* The task function. */
void vTaskServo( void *pvParameters );
void vTaskMeasure( void *pvParameters );
void vTaskCommand( void *pvParameters );

/* functions */
void avance(int vitesse_moteur);
void tourne(int angle, int vitesse_moteur);

/* Inter-process Communication */
    /* Initialize Semaphore to signal new angle set */
SemaphoreHandle_t sem_new_angle;
SemaphoreHandle_t mutex_tab;

/* Shared Ressources */
unsigned int distance_sonar[180] = {300};
int current_angle;
MPU6050 mpu;
int16_t ax, ay, az;
int16_t gx, gy, gz;
char go_right = 0;
char go_left  = 0;
char go_backward = 0;

/* Define the strings that will be passed in as the task parameters.  These are
   defined const and off the stack to ensure they remain valid when the tasks are
   executing. */
const char *pcTextForTaskServo = "Task Servo is running\r\n";
const char *pcTextForTaskMeasure = "Task Measure is running\t\n";
const char *pcTextForTaskCommand = "Task Command is running\t\n";

/*-----------------------------------------------------------*/

void setup( void )
{
    Wire.begin();
    Serial.begin(115200);
    //Serial.println("Demarrage");


    servo_sonar.attach(SERVO_SONAR_PIN);                      // attaches the servo pin to the servo_sonar object 
    moteur_droit.attach(MOTEUR_DROIT_PIN);
    moteur_gauche.attach(MOTEUR_GAUCHE_PIN);

    moteur_droit.attach(MOTEUR_DROIT_PIN);
    moteur_gauche.attach(MOTEUR_GAUCHE_PIN);

    /* Initialize IMU */

    //Serial.println("Initialize MPU");
    mpu.initialize();
    //Serial.println(mpu.testConnection() ? "Connected" : "Connection failed");


    sem_new_angle = xSemaphoreCreateCounting(1, 0);

    /* Mutex to handle tab access */
    mutex_tab = xSemaphoreCreateMutex();

    /* Create the first task at priority 3 */
    xTaskCreate( vTaskServo, "Task Servo", 150, (void*)pcTextForTaskServo, 3, NULL );

    /* ... and the second task at priority 2.*/
    xTaskCreate( vTaskMeasure, "Task Measure", 150, (void*)pcTextForTaskMeasure, 2, NULL );


    xTaskCreate( vTaskCommand, "Task Command", 150, (void*)pcTextForTaskCommand, 1, NULL );

    /* Start the scheduler so our tasks start executing. */
    vTaskStartScheduler();

    Serial.println(F("Insufficient RAM"));
    for( ;; );
    //  return 0;
}
/*-----------------------------------------------------------*/


/* Angle Config */
#define ANGLE_MAX  160       // Travel of sonar servo
#define FRONT_ANGLE 20      // Angle of front obstacle detection
void vTaskServo( void *pvParameters )
{
    char *pcTaskName;
    TickType_t xLastWakeTime;

    pcTaskName = ( char * ) pvParameters;


    /* Initialize current_angle variable */
    current_angle = 0;

    char sens = 0; // Sens 0 : Positive, Sens 1 : Negative

    /* Print out the name of this task. */
    vPrintString( pcTaskName );

    /* Initialize Sonar Sensor */
    NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);     // NewPing setup of pins and maximum distance.

    /* Initialize variable with current tick count */
    xLastWakeTime = xTaskGetTickCount();

    for( ;; )
    {

        if(sens == 0){
            if(current_angle < ANGLE_MAX) current_angle+=5;
            else  sens = 1;
        }
        else{       // if sens == 1
            if(current_angle > 20)  current_angle -=5;
            else  sens = 0;
        }

        //servo_sonar.write(current_angle);

        //Serial.println(current_angle);
        /* Wait 15 MS for servo to set angle */
        //vTaskDelay((15L * configTICK_RATE_HZ) / 1000L);


        unsigned int uS ;//= sonar.ping(); // Send ping, get ping time in microseconds (uS).

        if(uS/(0.1*US_ROUNDTRIP_CM)!=0){
            // Try to hold Mutex
            /*if( xSemaphoreTake( mutex_tab, ( TickType_t ) 10 ) == pdTRUE )
            {
                //distance_sonar[current_angle] = (1-LPF_param) * distance_sonar[current_angle] + (LPF_param) * uS / (0.1*US_ROUNDTRIP_CM) ;// Renvoie distance_sonar pour angle_sonar
                distance_sonar[current_angle] = uS/(0.1*US_ROUNDTRIP_CM);
                //Serial.println(distance_sonar[current_angle]);
                // Release Mutex
                //xSemaphoreGive( mutex_tab );
            }*/

            vTaskDelayUntil( &xLastWakeTime, ( 400 / portTICK_PERIOD_MS ) );

            //xSemaphoreGive(sem_new_angle);


        }
    }
}
//------------------------------------------------------------------------------

void vTaskMeasure( void *pvParameters )
{
    char *pcTaskName;

    TickType_t xLastWakeTime;
    pcTaskName = ( char * ) pvParameters;

    int16_t ax, ay, az;
    int16_t gx, gy, gz;


    /* Print out the name of this task. */
    vPrintString( pcTaskName );

    /* Initialize variable with current tick count */
    xLastWakeTime = xTaskGetTickCount();

    for( ;; )
    {

        //Serial.print("IMU-");
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        //int valy = map(ay, -17000, 17000, 0, 179);
        /*Serial.print(ax);
        Serial.print(",");
        Serial.print(ay);
        Serial.print(",");
        Serial.print(az);
        Serial.print(",");
        Serial.print(gx);
        Serial.print(",");
        Serial.print(gy);
        Serial.print(",");
        Serial.println(gz);*/
        //Serial.println(ay);
        if(ay < -7000){
            //Serial.println("Robot retourné !");
            go_backward == 1;
        }
        /* Wait for signal from thread Servo */
        //xSemaphoreTake(sem_new_angle, portMAX_DELAY);

        vTaskDelayUntil( &xLastWakeTime, ( 1000 / portTICK_PERIOD_MS ) );
        //vTaskDelay(500/portTICK_PERIOD_MS);
    }
}
//------------------------------------------------------------------------------
#define OBS_MIN 140
void vTaskCommand( void *pvParameters )
{
    char *pcTaskName;
    TickType_t xLastWakeTime;
    // FLAGS //
    unsigned int distance,dmin,dmax,dfront,avg_denom;

    pcTaskName = ( char * ) pvParameters;


    vPrintString( pcTaskName );
    xLastWakeTime = xTaskGetTickCount();

    for( ;; )
    {

        /*for(int angle =0;angle<ANGLE_MAX;angle+=5){
        // Try to hold Mutex
        }*/
        for(int i = 30;i<150;i+=5){
            if( xSemaphoreTake( mutex_tab, ( TickType_t ) 10 ) == pdTRUE )
            {
                if(distance_sonar[i]!=0){
                    distance = distance_sonar[i];}
                //Serial.println(distance);
                // Release Mutex
                xSemaphoreGive( mutex_tab );
            }
            if(distance < OBS_MIN){
                //Serial.print(i);
                //Serial.print(" - ");
                //Serial.print(distance_sonar[i]);
                //Serial.println("- OBSTACLE");
                if(i>110){
                    Serial.println("A DROITE !");
                    go_right = 1;
                    //tourne(5,4);
                }
                else if(i<70){
                    go_left = 1;
                    //tourne(-5,4);
                    Serial.println("A GAUCHE !");
                }
                else{
                    go_backward = 1;
                    //avance(-4);
                    //vTaskDelay(500/portTICK_PERIOD_MS);
                }
            }
            /*else{
                avance(4);
            }*/
            // Handle motor control
            if(go_backward == 1) {
                avance(-4);
                go_backward = 0;}
            else if(go_right == 1){
                tourne(5,4);
                go_right = 0;}
            else if(go_left == 1) {
                tourne(-5,4);
                go_left =0;}
            else avance(4);


        }


        vTaskDelayUntil( &xLastWakeTime, ( 500 / portTICK_PERIOD_MS ) );
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
        delay(abs(angle*9.5));
    }
    if( sens_rotation == -1)
    {
        moteur_droit.write((90 + vitesse_moteur*2.1) * sens_rotation) ;
        moteur_gauche.write((90 + vitesse_moteur) * sens_rotation) ;
        delay(abs(angle*6.5));
    }

    Serial.print("sens rotation  = ") ; Serial.println(sens_rotation) ;
    moteur_droit.write(90);
    moteur_gauche.write(90);

}


//------------------------------------------------------------------------------
void loop() {}


