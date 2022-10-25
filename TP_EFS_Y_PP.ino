#include <pitches.h>
#include <SFE_BMP180.h>
#include <Wire.h>

SFE_BMP180 bmp180;

#define BUZZER_PIN 16
#define LIGHT_SENSOR_PIN 34
#define MQ5_PIN 1
#define MQ7_PIN 2


//alarm vars
long alarmMils = 0;
int auxAlarm = 0;
bool activeAlarm = false;

// setting PWM properties
const int freq = 2000;
const int buzzerChannel = 0;
const int resolution = 8;

volatile int interruptCounter;
int totalInterruptCounter;

hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);


  if (activeAlarm) {
    alarmMils = alarmMils + 1;
  }


  portEXIT_CRITICAL_ISR(&timerMux);

}

void setup() {
  Serial.begin(115200);

  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 1000, true);
  timerAlarmEnable(timer);

  // configure LED PWM functionalitites
  ledcSetup(buzzerChannel, freq, resolution);

  // attach the channel to the GPIO to be controlled
  ledcAttachPin(BUZZER_PIN, buzzerChannel);
}


void loop() {
  checkBMP();
  checkLS();
  if(  checkMQ5() >= 800 && checkMQ7() >= 800){
    playAlarm();
  }
}

void playAlarm() {
  activeAlarm = true;
  switch (auxAlarm) {
    case 0:
      ledcWriteTone(buzzerChannel, NOTE_CS7);
      if (alarmMils == 500) {
        auxAlarm = 1;
        alarmMils = 0;
      }
      break;
    case 1:
      ledcWriteTone(buzzerChannel, NOTE_G7);
      if (alarmMils == 500) {
        auxAlarm = 0;
        alarmMils = 0;
      }
      break;
  }
}

int checkLS() {
  float ls = analogRead(LIGHT_SENSOR_PIN);
  int mapLS = map(ls, 0, 4095, 0, 100);
  Serial.println("Light Sensor: " + mapLS);
  return mapLS;
}

float checkMQ5() {
  float mq5 = analogRead(MQ5_PIN);
  Serial.println("Mq5: " + mq5);
  return mq5;
}
float checkMQ7() {
  float mq7 = analogRead(MQ7_PIN);
  Serial.println("Mq7: " + mq7);
  return mq7;
}

float checkBMP() {
  char status;
  double T, P;


  status = bmp180.startTemperature();//Inicio de lectura de temperatura
  if (status != 0)
  {
    delay(status); //Pausa para que finalice la lectura
    status = bmp180.getTemperature(T); //Obtener la temperatura
    if (status != 0)
    {
      status = bmp180.startPressure(3); //Inicio lectura de presión
      if (status != 0)
      {
        delay(status);//Pausa para que finalice la lectura
        status = bmp180.getPressure(P, T); //Obtenemos la presión
        if (status != 0)
        {
          Serial.print("Temperatura: ");
          Serial.print(T, 2);
          Serial.print(" *C , ");
          Serial.print("Presion: ");
          Serial.print(P, 2);
          Serial.println(" mb");
        }
      }
    }
  }
}
