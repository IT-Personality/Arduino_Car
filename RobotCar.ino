#include <Stepper.h>
#include <NewPing.h>

// для пинов управления моторами через драйвер L298N
const int MOTOR_LEFT_FORWARD = 9;
const int MOTOR_LEFT_BACKWARD = 10;
const int MOTOR_RIGHT_FORWARD = 11;
const int MOTOR_RIGHT_BACKWARD = 12;

// для ультразвукового датчика
#define ULTRASONIC_TRIG_PIN A0
#define ULTRASONIC_ECHO_PIN A1
#define MAX_DETECTABLE_DISTANCE 300

const int STEPPER_STEPS_PER_REV = 2048;
const int STEPPER_PIN_1 = 4;
const int STEPPER_PIN_2 = 5;
const int STEPPER_PIN_3 = 6;
const int STEPPER_PIN_4 = 7;
Stepper stepperMotor(STEPPER_STEPS_PER_REV, STEPPER_PIN_1, STEPPER_PIN_3, STEPPER_PIN_2, STEPPER_PIN_4);

// Глобальные переменные
int currentStepperPosition = 0; // Положение шагового двигателя
bool isMovingForward = false; // Флаг для отслеживания направления движения
int obstacleDistance = 100; // Текущее измеренное расстояние до препятствия
NewPing ultrasonicSensor(ULTRASONIC_TRIG_PIN, ULTRASONIC_ECHO_PIN, MAX_DETECTABLE_DISTANCE);


void setup() {
  pinMode(MOTOR_LEFT_FORWARD, OUTPUT);
  pinMode(MOTOR_LEFT_BACKWARD, OUTPUT);
  pinMode(MOTOR_RIGHT_FORWARD, OUTPUT);
  pinMode(MOTOR_RIGHT_BACKWARD, OUTPUT);

  // Настройка шагового двигателя
  stepperMotor.setSpeed(14);
  Serial.begin(9600);
  // Первичное измерение расстояния
  obstacleDistance = measureDistance();
  delay(100);
}

// Основной цикл работы
void loop() {
  int distanceToRight = 0;
  int distanceToLeft = 0;
  delay(50);

  if (obstacleDistance <= 30) {
    stopMotors();
    delay(200);

    moveBackward();
    delay(500);

    stopMotors();
    delay(300);

    distanceToRight = rotateStepperRight();
    delay(300);

    distanceToLeft = rotateStepperLeft();
    delay(300);

    if (distanceToRight >= distanceToLeft) {
      turnRight();
    } else {
      turnLeft();
    }
    stopMotors();
  } else {
    moveForward();
  }
  obstacleDistance = measureDistance();
}

// Функция измерения расстояния
int measureDistance() {
  delay(70);
  int distance = ultrasonicSensor.ping_cm();
  return (distance == 0) ? 250 : distance;
}

int rotateStepperRight() {
  stepperMotor.step(STEPPER_STEPS_PER_REV / 4);
  currentStepperPosition += STEPPER_STEPS_PER_REV / 4;
  delay(500);
  return measureDistance();
}

int rotateStepperLeft() {
  stepperMotor.step(-STEPPER_STEPS_PER_REV / 2);
  currentStepperPosition -= STEPPER_STEPS_PER_REV / 2;
  delay(500);

  int distance = measureDistance();

  stepperMotor.step(STEPPER_STEPS_PER_REV / 4);
  currentStepperPosition += STEPPER_STEPS_PER_REV / 4;
  delay(100);
  return distance;
}

// Остановка всех моторов
void stopMotors() {
  digitalWrite(MOTOR_LEFT_FORWARD, LOW);
  digitalWrite(MOTOR_RIGHT_FORWARD, LOW);
  digitalWrite(MOTOR_LEFT_BACKWARD, LOW);
  digitalWrite(MOTOR_RIGHT_BACKWARD, LOW);
}

// Движение вперед
void moveForward() {
  if (!isMovingForward) {
    isMovingForward = true;
    digitalWrite(MOTOR_LEFT_FORWARD, HIGH);
    digitalWrite(MOTOR_RIGHT_FORWARD, HIGH);
    digitalWrite(MOTOR_LEFT_BACKWARD, LOW);
    digitalWrite(MOTOR_RIGHT_BACKWARD, LOW);
  }
}

// Движение назад
void moveBackward() {
  isMovingForward = false;
  digitalWrite(MOTOR_LEFT_BACKWARD, HIGH);
  digitalWrite(MOTOR_RIGHT_BACKWARD, HIGH);
  digitalWrite(MOTOR_LEFT_FORWARD, LOW);
  digitalWrite(MOTOR_RIGHT_FORWARD, LOW);
}

// Поворот направо
void turnRight() {
  digitalWrite(MOTOR_LEFT_FORWARD, HIGH);
  digitalWrite(MOTOR_RIGHT_BACKWARD, HIGH);
  digitalWrite(MOTOR_LEFT_BACKWARD, LOW);
  digitalWrite(MOTOR_RIGHT_FORWARD, LOW);
  delay(1500);
}

// Поворот налево
void turnLeft() {
  digitalWrite(MOTOR_LEFT_BACKWARD, HIGH);
  digitalWrite(MOTOR_RIGHT_FORWARD, HIGH);
  digitalWrite(MOTOR_LEFT_FORWARD, LOW);
  digitalWrite(MOTOR_RIGHT_BACKWARD, LOW);
  delay(1500);
}