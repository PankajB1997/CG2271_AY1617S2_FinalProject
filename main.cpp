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
#define TICKS_TO_WAIT 250

QueueHandle_t distanceQueue = NULL;
QueueHandle_t driverDesiredSpeedQueue = NULL;
QueueHandle_t currentSpeedQueue = NULL;
QueueHandle_t checkSpeedQueue = NULL;
QueueHandle_t csQueue = NULL;
QueueHandle_t dsQueue = NULL;
QueueHandle_t distQueue = NULL;

static unsigned long previousInterruptTime1 = 0;
static unsigned long previousInterruptTime2 = 0;

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

void increaseSpeed(int safeSpeed, int automatedSafetyFeature,
		int *driverDesiredSpeed, int *currentSoundLevelAndSpeed) {

	if (*currentSoundLevelAndSpeed < 3 && !automatedSafetyFeature) {
		currentSoundLevelAndSpeed = currentSoundLevelAndSpeed + 1;
		*driverDesiredSpeed = *driverDesiredSpeed + 1;
		if (*driverDesiredSpeed > 3)
			*driverDesiredSpeed = 3;
	}
	if (automatedSafetyFeature)
		*currentSoundLevelAndSpeed = safeSpeed;
	playBuzzer(*currentSoundLevelAndSpeed);
	lightLEDs(*currentSoundLevelAndSpeed);
}

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

void readDistanceTask(void *p) {
	TickType_t xLastWakeTime = 0;
	const TickType_t xFrequency = 500;
	int distance, desiredSpeed = 0, currentSpeed = 0;
	for (;;) {
		distance = analogRead(PIN_PTTM);
//		xQueueReceive(csQueue, (void * ) &currentSpeed, (TickType_t ) 0);
//		xQueueReceive(dsQueue, (void * ) &desiredSpeed, (TickType_t ) 0);
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
			xQueueOverwrite(dsQueue, (void * ) &desiredSpeed);
		}
		xQueueOverwrite(distanceQueue, (void * ) &distance);
		xQueueOverwrite(distQueue, (void * ) &distance);
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
}

void checkSpeedTask(void *p) {
	TickType_t xLastWakeTime = 0;
	const TickType_t xFrequency = 1000;
	int flag, distance, desiredSpeed = 0, currentSpeed = 0;
	for (;;) {
		if ( xQueueReceive(checkSpeedQueue, (void * ) &flag,
				(TickType_t ) 0) == pdTRUE) {
			xQueueReceive(csQueue, (void * ) &currentSpeed, (TickType_t ) 0);
			xQueueReceive(dsQueue, (void * ) &desiredSpeed, (TickType_t ) 0);
			xQueueReceive(distQueue, (void * ) &distance, (TickType_t ) 0);
			if (flag == 512) {
				increaseSpeed(-1, 0, &desiredSpeed, &currentSpeed);
			} else if (flag == 770) {
				decreaseSpeed(&desiredSpeed, &currentSpeed);
			}
			if (flag == 512 && (currentSpeed > getSafeSpeed(distance))) {
				digitalWrite(RED_LED, HIGH);
				increaseSpeed(getSafeSpeed(distance), 1, &desiredSpeed,
						&currentSpeed);
			}
			xQueueOverwrite(currentSpeedQueue, (void * ) &currentSpeed);
			xQueueOverwrite(driverDesiredSpeedQueue, (void * ) &desiredSpeed);
			xQueueOverwrite(csQueue, (void * ) &currentSpeed);
			xQueueOverwrite(dsQueue, (void * ) &desiredSpeed);
		}
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
		digitalWrite(RED_LED, LOW);
	}
}
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

void btn1ISR() {
	static signed char xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;
	unsigned long currentTime;
	currentTime = millis();
	if (currentTime - previousInterruptTime1 >= 250) {
		xQueueOverwrite(checkSpeedQueue, (void * ) 1);
		previousInterruptTime1 = currentTime;
	}

	if (xHigherPriorityTaskWoken) {
		taskYIELD();
	}
}

void btn2ISR() {
	static signed char xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;
	unsigned long currentTime;
	currentTime = millis();
	if (currentTime - previousInterruptTime2 >= 250) {
		xQueueOverwrite(checkSpeedQueue, (void * ) 2);
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

//	digitalWrite(RED_LED, HIGH);
//	digitalWrite(YELLOW_LED_1, HIGH);
//	digitalWrite(YELLOW_LED_2, HIGH);
//	digitalWrite(YELLOW_LED_3, HIGH);
//	analogWrite(SPEAKER, 16);

	distanceQueue = xQueueCreate(QUEUE_SIZE, sizeof(int));
	driverDesiredSpeedQueue = xQueueCreate(QUEUE_SIZE, sizeof(int));
	currentSpeedQueue = xQueueCreate(QUEUE_SIZE, sizeof(int));
	checkSpeedQueue = xQueueCreate(1, sizeof(int));
	csQueue = xQueueCreate(1, sizeof(int));
	dsQueue = xQueueCreate(1, sizeof(int));
	distQueue = xQueueCreate(1, sizeof(int));

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