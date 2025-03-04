#include <Arduino.h>
#include "definitions.h"
#include <Wire.h>
static void controlTask(void *pvParameters);
static void controlPidTask(void *pvParameters);
static void serialTask(void *pvParameters);
// tiempo de muestreo dejar en 25ms
float h = 0.025;



// offset de calibracion para detectar la distancia en positivo con 0 en el 
//piso del tubo
float calibration = 100;


//limites del actuador
float umax = 9;//8.;
float umin = 5;

// variables del lazo de control
float reference = 50;
float y;
float u;
float usat;
float e;

float b = 80.0;
float L = 0.574;
float tau1 = 0.4328;
//float kp = (0.37 / (b * L)) + (0.02 * tau1 / (b *L*L));
//float ki = (0.03 / (b * (L*L))) + (0.0012 * tau1 / (b*L*L*L));
//float kd = (0.16 / b) + (0.28 * tau1 / (b * L));

float kp = 0.0066371;
float ki = 0.00060231;
float kd = 0.0048887;

/*
float kp = 0.01;//0.01;//0.01; //0.04
float ki = 0.01;//0.0005;
float kd = 0.01;//0.005;
*/
float deadzone = 0;

boolean reset_int = 0;

// valor de equilibrio de la señal de control
float ueq = 5.75;//6.325;

void setup() {
    // iniciamos el sensor  
    Serial.begin(115200);
    Wire.begin();
    while(!setupSensor()){
      vTaskDelay(1000);
    }
    setupPwm();    
    vTaskDelay(100);
    
    // Asi definimos la tarea de control, de la máxima prioridad en el nucleo 1
    xTaskCreatePinnedToCore(
        controlPidTask, // nombre de la rutina
            "general controller task",
            8192,
            NULL,
            23, // prioridad de la tarea (0-24) , siendo 24 la prioridad más critica      
            NULL,
            CORE_1
    );  
    
    xTaskCreatePinnedToCore(
        serialTask, 
        "serial task",
        8192,
        NULL,
        1,  // Baja prioridad para no interferir con PID
        NULL,
        CORE_0
    );

}


/***************************************************************************
*                CONTROL PROPORCIONAL DEL LEVITADOR
***************************************************************************/ 


static void controlTask(void *pvParameters) {

    // Aqui configuro cada cuanto se repite la tarea
    const TickType_t taskInterval = 1000*h;  // repetimos la tarea cada tiempo de muestreo en milisegundos = 1000*0.025= 25ms
    
    // prototipo de una tarea repetitiva   
    for (;;) {
       TickType_t xLastWakeTime = xTaskGetTickCount();

       // distancia en centimetros       
       y = calibration - (float) sensor.readRangeContinuousMillimeters()/10 ; 
       
       // control proporcional
       e = reference - y;
       u = kp*e + ueq;
       
       // control proporcional
       usat = constrain(u, umin, umax);
       voltsToFan(usat);
       //printf("%0.2f, %0.2f, %0.3f\n", reference, y, movingAverage(usat));
       
       // la tarea es crítica entonces esperamos exactamente taskInterval ms antes de activarla nuevamente
       vTaskDelayUntil(&xLastWakeTime, taskInterval);     

    }

}

/***************************************************************************
*                CONTROL PID DEL LEVITADOR
***************************************************************************/ 

static void controlPidTask(void *pvParameters) {
    /* this function computes a two parameter PID control with Antiwindup
     See Astrom ans Murray
    */
    static bool led_status = false;
    const TickType_t taskPeriod = (pdMS_TO_TICKS(1000*h));

    float bi;      //scaled integral constant
    float ad;      //scale derivative constant 1
    float bd;      //scale derivative constant 2
    float br =h/(0.9*h); // Taw = 0.99
    float P;       //  proportional action
    float D;       //  derivative action

    /** state variables for the PID */
    static float y_ant = 0;      //  past output
    static float I = 0;          // Integral action
    float e;
    float v;


    float beta = 0.7;//0.7;
    float N = 5;
    for (;;) {
        TickType_t xLastWakeTime = xTaskGetTickCount();
        // at start we reset the integral action and the state variables
        // when receiving a new command
        reference = reference;

        // distancia en centimetros       
        y = calibration - (float) sensor.readRangeContinuousMillimeters()/10 ; 
        
        // integral action scaled to sampling time
        bi = ki*h;
        // filtered derivative constant
        ad = kd/(N*(kd/N + h));
        bd = kd/(kd/N + h);
        P = kp*(beta * reference - y); // proportional actions
        D =  ad*D - bd*(y - y_ant); // derivative action
        u = P + I +  D ; // control signal
        e = reference - y;
        v = (y - y_ant)/h;


        //saturated control signal
        usat =  constrain(u, umin, umax);
        voltsToFan((usat));
        printf("ref = %0.2f y =  %0.2f usat =  %0.3f P = %0.2f I = %0.2f D = %0.2f\n", reference, y, movingAverage(usat),P,I,D);
        // updating integral action
        if (ki!= 0) {
            I = I + bi * e + br * (usat - u);
        } 
        if(reset_int){
            I = I;
            reset_int = false;
        }
        // updating output
        y_ant = y;
        //The task is suspended while awaiting a new sampling time
        vTaskDelayUntil(&xLastWakeTime, taskPeriod);    
        
        
    }
}

/***************************************************************************
*                ACTUALIZACIÓN REFERENCIA SERIAL
***************************************************************************/ 

static void serialTask(void *pvParameters) {
    Serial.println("Ingrese nueva referencia (10 - 90):");

    for (;;) {
        if (Serial.available() > 0) {
            String input = Serial.readStringUntil('\n'); 
            input.trim();
            if (input.length() > 0) {
                float newRef = input.toFloat(); 
                if (newRef >= 0 && newRef <= 100) { 
                    reference = newRef;
                    reset_int = true;
                } else {}
            }
        }
        vTaskDelay(100);
    }
}

void loop() {
    vTaskDelete(NULL);
}