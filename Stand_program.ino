/*..........................................ПОДКЛЮЧЕНИЕ...............................................................
-кнопка подключается одним контактом к минусу, вторым контактом на пин D3
*экран подключается SDA- А4, SCL- А5 (плюс и минус и так понятно)
*потенциометр задания оборотов подключается крайними ногами к плюсу и минусу соответсвтвенно (без разницы куда), средняя нога к А3
*тензодатчик DT- A0; SCK- A1 (плюс и минус и так понятно)
*сервомашинки от сопла- D11
*потенциометр сервомашинок А6
*к драйверу мотора- черный провод это минус. Сигнальный белый к D9
Оптический датчик- D2, плюс и минус и так понятно*/

/*..........................................ПРИНЦИП РАБОТЫ...........................................................
*Включаем сначала мотор
*Через несколько секунд после двух звуковых сигналов от мотора включаем ардуинку
*Ждем инициализацию
*как только появилось число слева сверху крутим потенциометр и выбираем обороты в минуту мотора
*нажимаем кнопку
*мотор раскручивается, идет подсчет оборотов (справа сверху), замер тяги в граммах (слева снизу), вычисление относительной площади сопло(справа снизу, !!!РЕЗУЛЬТАТ РАЗДЕЛИТЬ НА 1000!!!)
*после повторного нажатия на кнопку мотор постепенно останавливается и появляется возможность снова выбрать обороты */

#include <LiquidCrystal_I2C.h>  // подключаем библиотеку для работы с экраном
#include <Servo.h>              // подключаем библиотеку для работы с мотором
#include <GyverHX711.h>         // подключаем библиотеку для работы с тензодатчиком
#include "GyverPID.h"           // подключаем библиотеку для ПИД- регулятора
#include <Honeywell_SPI.h>      // подключаем библиотеку для работы с датчиками давления

GyverPID regulator(0.02, 0.03, 0.0, 50);  // коэф. П, коэф. И, коэф. Д, период дискретизации dt (мс)

int weight;
GyverHX711 sensor(A0, A1, HX_GAIN128_A);  //настройка работы тензодатчика

LiquidCrystal_I2C lcd(0x27, 16, 2);  //настраиваем работу экрана

float i;  //переменная для задания количества оборотов мотора

volatile boolean flag;  //переменная для работы с кнопкой

Servo motor;          //переменная для мотора
#define mot_pin 9     //пин для подключения сигнального провода мотора
#define max_pwm 2300  //Максимальное значение ШИМ 2.3 мс
#define min_pwm 800   //Минимальное значени ШИМ 0.8 мс

Servo soplo;         //переменная для сопла
#define pinServo 10   // Пин для подключения сервопривода
#define POT 6        // Аналоговый вход A6 для подключения потенциометра
int F;               // переменная для хранения относительной площади выходного сечения сопла

volatile float rpm;           //обороты в минуту
volatile unsigned long Time;  //переменная для подсчета оборотов

Honeywell_SPI pressureStatic(8, 0.915541, -6000, 1638, 10);  // (SS пин, константа, минимальное давление, минимальное значение с датчика,кол-во измерений в одной точке)----Константа равна = (maxPressure-minPressure) / (outputMax - outputMin)
Honeywell_SPI pressureTotal(7, 0.915541, -6000, 1638, 10);    //второй датчик
float Pst;
float Pdin;
float Pt;
float temp;
float thrust_aero;
float massFlow;
#define density 1.2

void setup() {
  Serial.begin(9600);
  lcd.init();           // инициализация экрана
  lcd.backlight();      // включить подсветку экрана
  lcd.setCursor(1, 0);  //без комментариев
  lcd.print("STAND");
  lcd.setCursor(1, 1);
  lcd.print("INITIALIZATION");
  regulator.setDirection(NORMAL);              // направление регулирования (NORMAL/REVERSE). ПО УМОЛЧАНИЮ СТОИТ NORMAL
  regulator.setLimits(min_pwm, max_pwm);       // пределы (ставим для 8 битного ШИМ). ПО УМОЛЧАНИЮ СТОЯТ 0 И 255
  pinMode(3, INPUT_PULLUP);                    // инициируем работу кнопки
  pinMode(A3, INPUT);                          // инициируем работу потенциометра
  soplo.attach(pinServo);                      // инициируем работу потенциометра сопла
  motor.attach(mot_pin);                       //  инициализация пина мотора
  soplo.write(27);                             //установка положения сопла с F=0
  attachInterrupt(0, rpmTime, FALLING);        //аппаратное прерывание для датчика H206
  attachInterrupt(1, buttInterrupt, FALLING);  //аппаратное прерывание для кнопки
  motor.writeMicroseconds(max_pwm);            //калибровка мотора (включить максимальный ШИМ, затем минимальный)
  delay(3000);
  motor.writeMicroseconds(min_pwm);
  delay(5000);
  sensor.tare();  // калибровка нуля тензодатчика
  flag = 0;
  pressureStatic.begin();  //инициализация датчика 1
  pressureTotal.begin();   //инициализация датчика 2
}

void loop() {  //основной цикл, пока кнопка не нажата выбираем обороты потенциметром
  if (flag == 0) {
    setRPM();
  } else {
    regulator.setpoint = i;                               // сообщаем регулятору температуру, которую он должен поддерживать
    regulator.input = rpm;                                // сообщаем регулятору текущую температуру
    motor.writeMicroseconds(regulator.getResultTimer());  //исходя из данных датчика, подача сигнала мотору
    thrust();                                             //замер тяги
    soploR();                                             //сопло
    LCD();                                                //вывод значений на экран
    read_aero();
    COM_port();
  }
}


void setRPM() {  //алгоритм задания оборотов (крутим потенциометр- на экране обороты в минуту)
  rpm = 0.0;
  motor.writeMicroseconds(min_pwm);
  i = analogRead(3) * 20;  //чтение значения с потенциометра А3
  LCD();
}


void thrust() {  //алгоритм замера тяги с тензодатчика
  weight = sensor.read() / (-1000.0);
}


void LCD() {  //вывод значений на экран
  static uint32_t tmr;
  if (millis() - tmr >= 200) {
    tmr = millis();
    lcd.clear();
    lcd.home();            // задание начальных координат надписи
    lcd.print("S");        // Set- выбранные обороты, об/мин
    lcd.setCursor(10, 0);  // координаты надписи дейтсвительных оборотов мотора
    lcd.print("V");        // Valid- действительных оборотов двигателя, об/мин
    lcd.setCursor(0, 1);   // координаты надписи тяги в граммах
    lcd.print("T");        // Thrust- тягя в граммах
    lcd.setCursor(10, 1);  // координаты надписи относительной площади на выходном сечении сопла
    lcd.print("F");        // F- относительная площадь на выходном сечении сопла
    lcd.setCursor(11, 0);  // координаты надписи дейтсвительных оборотов мотора
    lcd.print(rpm);
    lcd.setCursor(1, 0);  //вывод значения на экран
    lcd.print(i);
    lcd.setCursor(11, 1);  // координаты надписи дейтсвительных оборотов мотора
    lcd.print(F);
    lcd.setCursor(1, 1);  // координаты надписи массы
    lcd.print(weight);
  }
}


void soploR() {                                        //управление соплом
  int angleServo;                                      // переменная для хранения угла поворота сервы
  angleServo = map(analogRead(POT), 0, 1023, 22, 60);  // масштабируем значение к интервалу 0-180
  soplo.write(angleServo);                             // поворот сервопривода на полученный угол
  F = map(angleServo, 22, 60, 1100, 300);
}


void rpmTime() {  //аппаратное прерываение, подсчет времени и вычисление оборотов в минуту
  rpm = 60000.0 / (millis() - Time);
  Time = millis();
}


void buttInterrupt() {  //аппаратное прерывание кнопки
  flag = !flag;
}


void read_aero() {
  static uint32_t tmr_2;
  if (millis() - tmr_2 >= 600) {
    tmr_2 = millis();
    pressureStatic.readSensor();
    Pst = pressureStatic.getPressure();      //статическое давление
    temp = pressureStatic.getTemperature();  //температура
    pressureTotal.readSensor();
    Pt = pressureTotal.getPressure();          //полное давление
    Pdin = (Pt - Pst);                         //динамическое давление
    thrust_aero = Pdin * F * 2 * 0.001 * 0.010053;                //тяга
    massFlow = F * 0.001 * 0.010053 * sqrtf(2 * Pdin * density);  //расход
  }
}


void COM_port() {
  static uint32_t tmr_3;
  if (millis() - tmr_3 >= 1000) {
    tmr_3 = millis();
    Serial.print("Ptot: " + String(Pt) + " Pa; Pst: ");
    Serial.print(String(Pst) + " Pa; aT: ");
    Serial.print(thrust_aero * 100);
    Serial.print(" g; temp: ");
    Serial.print(String(temp) + " C; RPM: ");
    Serial.print(String(rpm) + " ; sT: ");
    Serial.print(weight);
    Serial.print(" g; G: ");
    Serial.print(massFlow, 4);
    Serial.println(" kg/s.");
  }
}