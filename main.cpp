/**
 * CG2271 Lab 6
 * AY2016-17 Semester 2
 * Students:
 * 1. PANKAJ BHOOTRA (A0144919W)
 * 2. CHAN JUN XUN (A0139313L)
 */

#include <Arduino.h>
#include <avr/io.h>
#include <FreeRTOS.h>
#include <semphr.h>
#include <task.h>
#include <queue.h>

#define STACK_SIZE 100
#define QUEUE_SIZE 2
#define RED_LED 6
#define YELLOW_LED_1 7
#define YELLOW_LED_2 8
#define YELLOW_LED_3 9
#define PIN_PTTM 0
#define BTN_1 2
#define BTN_2 3
#define SPEAKER 10

#define D 256

QueueHandle_t distanceQueue = NULL;
QueueHandle_t driverDesiredSpeedQueue = NULL;
QueueHandle_t currentSpeedQueue = NULL;
QueueHandle_t checkSpeedQueue = NULL;
QueueHandle_t distQueue = NULL;
QueueHandle_t ddsQueue = NULL;
QueueHandle_t csQueue = NULL;

static unsigned long previousInterruptTime1 = 0;
static unsigned long previousInterruptTime2 = 0;

/* Plays the buzzer whose volume is proportional to the speed of the car */
void playBuzzer(int soundLevel) {
	if (soundLevel == 1) {
		analogWrite(SPEAKER, 4);
	} else if (soundLevel == 2) {
		analogWrite(SPEAKER, 8);
	} else if (soundLevel == 3) {
		analogWrite(SPEAKER, 16);
	} else {
		analogWrite(SPEAKER, 2);
	}
}

/* Lights up the 3 yellow LEDs; the number of yellow LEDs lit up is the current speed of the car */
void lightLEDs(int soundLevel) {
	if (soundLevel == 1) {
		digitalWrite(YELLOW_LED_1, HIGH);
		digitalWrite(YELLOW_LED_2, LOW);
		digitalWrite(YELLOW_LED_3, LOW);
	} else if (soundLevel == 2) {
		digitalWrite(YELLOW_LED_1, HIGH);
		digitalWrite(YELLOW_LED_2, HIGH);
		digitalWrite(YELLOW_LED_3, LOW);
	} else if (soundLevel == 3) {
		digitalWrite(YELLOW_LED_1, HIGH);
		digitalWrite(YELLOW_LED_2, HIGH);
		digitalWrite(YELLOW_LED_3, HIGH);
	} else {
		digitalWrite(YELLOW_LED_1, LOW);
		digitalWrite(YELLOW_LED_2, LOW);
		digitalWrite(YELLOW_LED_3, LOW);
	}
}

/* fetches the maximum safe speed for a given value of distance */
int getSafeSpeed(int distance) {
	if (distance <= D)
		return 0;
	else if (distance <= (2 * D))
		return 1;
	else if (distance <= (3 * D))
		return 2;
	else
		return 3;
}

/* increases speed by 1 unit and also resets the speed to safe speed value if automated safety is ON */
void increaseSpeed(int safeSpeed, int automatedSafetyFeature,
		int *driverDesiredSpeed, int *currentSoundLevelAndSpeed) {

	if (*currentSoundLevelAndSpeed < 3 && !automatedSafetyFeature) {
		*currentSoundLevelAndSpeed = *currentSoundLevelAndSpeed + 1;
		*driverDesiredSpeed = *driverDesiredSpeed + 1;
		if (*driverDesiredSpeed > 3)
			*driverDesiredSpeed = 3;
	}
	if (automatedSafetyFeature)
		*currentSoundLevelAndSpeed = safeSpeed;
	playBuzzer(*currentSoundLevelAndSpeed);
	lightLEDs(*currentSoundLevelAndSpeed);
}

/* decreases speed by 1 unit */
void decreaseSpeed(int *driverDesiredSpeed, int *currentSoundLevelAndSpeed) {
	if (*currentSoundLevelAndSpeed > 0) {
		*currentSoundLevelAndSpeed = *currentSoundLevelAndSpeed - 1;
		*driverDesiredSpeed = *driverDesiredSpeed - 1;
		if (*driverDesiredSpeed < 0)
			*driverDesiredSpeed = 0;
		playBuzzer(*currentSoundLevelAndSpeed);
		lightLEDs(*currentSoundLevelAndSpeed);
	}
}

/* this task reads in the distance value periodically every 500 ms, also resets the speed to safe speed if the current speed is more than the maximum safe speed for this distance */
void readDistanceTask(void *p) {
	TickType_t xLastWakeTime = 0;
	const TickType_t xFrequency = 500;
	int distance, desiredSpeed = 0, currentSpeed = 0;
	for (;;) {
		distance = analogRead(PIN_PTTM);
		xQueueOverwrite(distQueue, (void * ) &distance);
		xQueuePeek(ddsQueue, (void * ) &desiredSpeed, (TickType_t ) 0);
		xQueuePeek(csQueue, (void * ) &currentSpeed, (TickType_t ) 0);
		if (currentSpeed > getSafeSpeed(distance)) {
			digitalWrite(RED_LED, HIGH);
			increaseSpeed(getSafeSpeed(distance), 1, &desiredSpeed,
					&currentSpeed);
			xQueueOverwrite(currentSpeedQueue, (void * ) &currentSpeed);
			xQueueOverwrite(csQueue, (void * ) &currentSpeed);
		}
		if (getSafeSpeed(distance) >= currentSpeed) {
			desiredSpeed = currentSpeed;
			xQueueOverwrite(driverDesiredSpeedQueue, (void * ) &desiredSpeed);
			xQueueOverwrite(ddsQueue, (void * ) &desiredSpeed);
		}
		xQueueOverwrite(distanceQueue, (void * ) &distance);
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
}

/* this task periodically runs every 1000 ms; increases/decreases speed if the relevant pushbutton was pressed, also resets the speed to safe speed if current speed is being increased beyond the safe speed for current distance */
void checkSpeedTask(void *p) {
	TickType_t xLastWakeTime = 0;
	const TickType_t xFrequency = 1000;
	int flag, distance, desiredSpeed = 0, currentSpeed = 0;
	for (;;) {
		if ( xQueueReceive(checkSpeedQueue, (void * ) &flag,
				(TickType_t ) 0) == pdTRUE) {
			xQueuePeek(ddsQueue, (void * ) &desiredSpeed, (TickType_t ) 0);
			xQueuePeek(csQueue, (void * ) &currentSpeed, (TickType_t ) 0);
			xQueueReceive(distQueue, (void * ) &distance, (TickType_t ) 0);
			if (flag == 1) {
				increaseSpeed(-1, 0, &desiredSpeed, &currentSpeed);
			} else if (flag == 2) {
				decreaseSpeed(&desiredSpeed, &currentSpeed);
			}
			if (flag == 1 && (currentSpeed > getSafeSpeed(distance))) {
				digitalWrite(RED_LED, HIGH);
				increaseSpeed(getSafeSpeed(distance), 1, &desiredSpeed,
						&currentSpeed);
			}
			xQueueOverwrite(currentSpeedQueue, (void * ) &currentSpeed);
			xQueueOverwrite(csQueue, (void * ) &currentSpeed);
			xQueueOverwrite(driverDesiredSpeedQueue, (void * ) &desiredSpeed);
			xQueueOverwrite(ddsQueue, (void * ) &desiredSpeed);
		}
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
		digitalWrite(RED_LED, LOW);
	}
}

/* this task periodically prints the distance, driver desired speed and current speed every 500 ms */
void printTask(void *p) {
	TickType_t xLastWakeTime = 0;
	const TickType_t xFrequency = 500;
	int dVal = 0, ddsVal = 0, csVal = 0;
	for (;;) {
		xQueueReceive(distanceQueue, (void * ) &dVal, (TickType_t ) 0);
		xQueueReceive(currentSpeedQueue, (void * ) &csVal, (TickType_t ) 0);
		xQueueReceive(driverDesiredSpeedQueue, (void * ) &ddsVal,
				(TickType_t ) 0);
		Serial.print("Desired speed: ");
		Serial.print(ddsVal);
		Serial.print(", Current speed: ");
		Serial.print(csVal);
		Serial.print(", Distance: ");
		Serial.println(dVal);
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
}

/* Interrupt handler for increase speed pushbutton */
void btn1ISR() {
	static signed char xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;
	unsigned long currentTime;
	currentTime = millis();
	if (currentTime - previousInterruptTime1 >= 250) {
		xQueueOverwriteFromISR(checkSpeedQueue, (void * ) 1, &xHigherPriorityTaskWoken);
		previousInterruptTime1 = currentTime;
	}

	if (xHigherPriorityTaskWoken) {
		taskYIELD();
	}
}

/* Interrupt handler for decrease speed pushbutton */
void btn2ISR() {
	static signed char xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;
	unsigned long currentTime;
	currentTime = millis();
	if (currentTime - previousInterruptTime2 >= 250) {
		xQueueOverwriteFromISR(checkSpeedQueue, (void * ) 2, &xHigherPriorityTaskWoken);
		previousInterruptTime2 = currentTime;
	}

	if (xHigherPriorityTaskWoken) {
		taskYIELD();
	}
}

void setup() {
	Serial.begin(115200);
	pinMode(RED_LED, OUTPUT);
	pinMode(YELLOW_LED_1, OUTPUT);
	pinMode(YELLOW_LED_2, OUTPUT);
	pinMode(YELLOW_LED_3, OUTPUT);
	pinMode(SPEAKER, OUTPUT);
	pinMode(BTN_1, INPUT);
	pinMode(BTN_2, INPUT);

	attachInterrupt(0, btn1ISR, FALLING);
	attachInterrupt(1, btn2ISR, FALLING);

	distanceQueue = xQueueCreate(QUEUE_SIZE, sizeof(int));
	driverDesiredSpeedQueue = xQueueCreate(QUEUE_SIZE, sizeof(int));
	currentSpeedQueue = xQueueCreate(QUEUE_SIZE, sizeof(int));
	checkSpeedQueue = xQueueCreate(QUEUE_SIZE, sizeof(int));
	distQueue = xQueueCreate(QUEUE_SIZE, sizeof(int));
	ddsQueue = xQueueCreate(QUEUE_SIZE, sizeof(int));
	csQueue = xQueueCreate(QUEUE_SIZE, sizeof(int));

	playBuzzer(0);
	lightLEDs(0);
}

void loop() {
	xTaskCreate(readDistanceTask, "ReadDistanceTask", STACK_SIZE, NULL, 1,
			NULL);
	xTaskCreate(checkSpeedTask, "CheckSpeedTask", STACK_SIZE, NULL, 1, NULL);
	xTaskCreate(printTask, "PrintTask", STACK_SIZE, NULL, 1, NULL);
	/* start scheduler */
	vTaskStartScheduler();
}
