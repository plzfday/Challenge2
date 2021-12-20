#include <Wire.h>

#define MASTER_ADDRESS 9
#define SIZE 16
#define PID_BUFFER_SIZE 10

const int Acid = 5;
const int Base = 6;
const int phPin = A0;
const int Thermistor = A1;
const int Heater = 9;
const int lightsen = A2;
const int Motor = 10;

int phInput = 0; // initialise the pins
float phValue = 0;
int tempIn = 0;

float temperature = 0;
float R1 = 10000; // connect this resistor with thermistor in series
float logR2, R2, T;
float c1 = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07;
int stirIn = 0;
int stirrate = 0;

int phsensorMin = 1023;
int phsensorMax = 0;
int tempsensorMin = 1023;
int tempsensorMax = 0;
int stirMin = 1023;
int stirMax = 0;
// parameter for stirring detection
int previousValue;
unsigned long previousTime;
int sensor1_laststate = 0;
int sensor2_laststate = 0;
int sensor1_state = 0;
int sensor2_state = 0;
int count = 0;
int lastcounter = 0;
double motorStirSpeed;

int lastTime = 0;

float desiredTemperature; // initialise  desired values
float desiredPH;
float desiredStirring;

int heatstatus = 0; // initialise status of subsystem
int stirringstatus = 0;
int phstatus = 0;

struct PIDControl
{
    // int n;
    // int values[PID_BUFFER_SIZE];
    // int times[PID_BUFFER_SIZE];

    double ki;
    double kd;
    double kp;

    double error;
    double cumError, rateError;
    double lastError;

    int lastTime;
};

void update_pid(struct PIDControl *pid, double val, double desired)
{
    int t = millis() - pid->lastTime;
    pid->error = desired - val;
    pid->cumError += pid->error * t;
    pid->rateError = (pid->error - pid->lastError) / t;
    pid->lastTime = millis();
    pid->lastError = pid->error;
}

double get_value(struct PIDControl *pid)
{
    return pid->ki * pid->cumError + pid->kd * pid->rateError + pid->kp * pid->error;
}

struct PIDControl temperatureControl;
struct PIDControl PHControl;
struct PIDControl StirringControl;

void setup()
{
    // put your setup code here, to run once:
    Serial.begin(115200);
    Wire.begin(MASTER_ADDRESS);
    pinMode(Motor, OUTPUT);
    pinMode(Heater, OUTPUT);
    pinMode(Acid, OUTPUT);
    pinMode(Base, OUTPUT);
    pinMode(phPin, INPUT);
    pinMode(Thermistor, INPUT);
    pinMode(lightsen, INPUT);
    previousValue = analogRead(lightsen);
    previousTime = millis();

    // Init the PID controls
    memset(&temperatureControl, 0, sizeof(temperatureControl));
    temperatureControl.ki = 0.5;
    temperatureControl.kd = 0.5;
    temperatureControl.kp = 0.5;

    memset(&PHControl, 0, sizeof(PHControl));
    PHControl.ki = 0.5;
    PHControl.kd = 0.5;
    PHControl.kp = 0.5;

    memset(&StirringControl, 0, sizeof(StirringControl));
    StirringControl.ki = 0.5;
    StirringControl.kd = 0.5;
    StirringControl.kp = 0.5;

    // initialize connectivity
    Wire.onReceive(receiveEvent);
    Wire.onRequest(requestEvent);

    analogWrite(Motor, 50);
    while (millis() < 5000)
    {
        // calibration loop
        phInput = analogRead(phPin);
        tempIn = analogRead(Thermistor);
        stirIn = analogRead(lightsen);

        // record the maximum sensor value
        if (phInput > phsensorMax)
        {
            phsensorMax = phInput;
        }

        // record the minimum sensor value
        if (phInput < phsensorMin)
        {
            phsensorMin = phInput;
        }

        if (tempIn > tempsensorMax)
        {
            tempsensorMax = tempIn;
        }

        if (tempIn < tempsensorMin)
        {
            tempsensorMin = tempIn;
        }

        //    // make sure that the stir sensor is positioned over the white bit
        //    stirThreshold = stirIn;

        // Auto threshold
        if (stirIn > stirMax)
        {
            stirMax = stirIn;
        }

        if (stirIn < stirMin)
        {
            stirMin = stirIn;
        }
    }

    analogWrite(Motor, 0);

    lastTime = millis();
    temperatureControl.lastTime = lastTime;
    PHControl.lastTime = lastTime;
    StirringControl.lastTime = lastTime;
}

// Slave receiver
void receiveEvent()
{
    String receive = "";
    while (Wire.available())
    {
        char c = Wire.read();
        receive += c;
    }

    char type;
    type = receive[0]; // get desired temperature
    String svalue = receive.substring(1);
    //  Serial.print("Type:");
    //  Serial.print(type);
    //  Serial.print(" Svalue:");
    //  Serial.print(svalue);
    //  Serial.print(" Conversion:");
    //  Serial.print(svalue.toFloat());
    //  Serial.println("\n==============");
    if (type == '3')
    {
        desiredTemperature = svalue.toFloat();
    }
    else if (type == '4')
    {
        desiredPH = svalue.toFloat();
    }
    else if (type == '5')
    {
        desiredStirring = svalue.toFloat();
    }
}
// slave sender

void sendToArduinoFloat(char type, float value)
{
    char buf[SIZE + 1];
    String temp = type + String(value, 1) + "\n";
    temp.toCharArray(buf, SIZE + 1);
    Wire.write(buf);
}

void sendToArduinoInt(char type, int value)
{
    char buf[SIZE + 1];
    String temp = type + String(value) + "\n";
    Serial.println(temp);
    temp.toCharArray(buf, SIZE + 1);
    Wire.write(buf);
}

void requestEvent()
{
    sendToArduinoFloat('0', temperature);
    sendToArduinoFloat('1', phValue);
    sendToArduinoFloat('2', motorStirSpeed);
    sendToArduinoInt('6', heatstatus);
    sendToArduinoInt('7', phstatus);
    sendToArduinoInt('8', stirringstatus);
}

void loop()
{
    int currentTime = millis();

    // get input from different sensors
    phInput = analogRead(phPin);
    phValue = map(phInput - 245, 90, -90, 1, 13);
//    float Voltage = phInput * (5 / 1023);
//    phValue = 7; // we should put the linear relationship here

    tempIn = analogRead(Thermistor);
    R2 = R1 * (1023.0 / (float)tempIn - 1.0);
    logR2 = log(R2);
    T = (1.0 / (c1 + c2 * logR2 + c3 * logR2 * logR2 * logR2));
    temperature = T - 273.15;

    // Stirring speed detection
    stirIn = analogRead(lightsen);
    int sensorstate_1 = analogRead(lightsen);
    delay(2);
    int sensorstate_2 = analogRead(lightsen);
    if (sensorstate_1 > 100)
    {
        sensor1_state = 1;
    }
    else
    {
        sensor1_state = 0;
    }
    if (sensorstate_2 > 100)
    {
        sensor2_state = 1;
    }
    else
    {
        sensor2_state = 0;
    }
    if (sensor1_state != sensor2_state)
    {
        count = count + 1;
    }
    if (count >= 4)
    {
        count = 0;

        motorStirSpeed = 60000.0 / (double(millis() - previousTime));
        previousTime = millis();
        //   Serial.println(previousTime);
    }

//     desiredTemperature=37.5;              //delete this when using
//     desiredStirring=500;
//     desiredPH=6.6;

    // Control different subsystems
    if (phValue < desiredPH)
    { // increase the ph
        update_pid(&PHControl, phValue, desiredPH);
        double phoutput = get_value(&PHControl);
        if (phoutput >= 255)
        {
            phoutput = 255;
        }
        analogWrite(Base, phoutput);
        phstatus = 1;
    }
    else
    {
        analogWrite(Base, 0);
        phstatus = 0;
    }

    if (phValue > desiredPH)
    { // lower the ph
        update_pid(&PHControl, phValue, desiredPH);
        double phoutput = get_value(&PHControl);
        if (phoutput >= 255)
        {
            phoutput = 255;
        }
        analogWrite(Acid, phoutput);
        phstatus = 1;
    }
    else
    {
        analogWrite(Acid, 0);
        phstatus = 0;
    }

    if (temperature < desiredTemperature-0.3)
    { // heating if temperature is low
        update_pid(&temperatureControl, temperature, desiredTemperature);
        double heatoutput = get_value(&temperatureControl);
        if (heatoutput >= 255)
        {
            heatoutput = 255;
        }
        analogWrite(Heater, heatoutput);
        heatstatus = 1;
    }
    else
    {
        analogWrite(Heater, 0);
        heatstatus = 0;
    }

    if ((motorStirSpeed < (desiredStirring-20))||(motorStirSpeed >(desiredStirring+20)))
    {
        update_pid(&StirringControl, motorStirSpeed, desiredStirring);
        double stirringoutput = get_value(&StirringControl);
        if (stirringoutput >= 255)
        {
            stirringoutput = 255;
        }
        analogWrite(Motor, stirringoutput); // the Motor should have desired speed written to it
        stirringstatus = 1;
    }
   else
    {
        stirringstatus = 0;
    }

//     Monitor
      String stringph = "current PH is: ";
      String prinph = stringph + phValue;
      String stringtem = "current temperature is: ";
      String printem = stringtem + temperature;
      String stringrpm = "current RPM is: ";
      String prinrpm = stringrpm + motorStirSpeed;
      Serial.println(prinph);
      Serial.println(printem);
      Serial.println(prinrpm);
      Serial.println(desiredStirring);
      Serial.println(desiredTemperature);
      Serial.println(desiredPH);

    lastTime = currentTime;
}
