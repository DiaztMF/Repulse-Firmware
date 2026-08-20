#include "BAT_Driver.h"
#include "Display_SPD2010.h"
#include "Gyro_QMI8658.h"
#include "I2C_Driver.h"
#include "LVGL_Driver.h"
#include "PWR_Key.h"
#include "RTC_PCF85063.h"
#include "Repulse_UI.h"
#include "Repulse_Link.h"
#include "Repulse_Storage.h"
#include "TCA9554PWR.h"

static char rtc_command[40];
static uint8_t rtc_command_length = 0;

static bool Is_Leap_Year(uint16_t year) {
  return (year % 4 == 0 && year % 100 != 0) || year % 400 == 0;
}

static bool Is_Valid_Date_Time(int year, int month, int day, int hour, int minute, int second) {
  static const uint8_t days_per_month[] = { 31, 28, 31, 30, 31, 30,
                                            31, 31, 30, 31, 30, 31 };
  if (year < 2024 || year > 2069 || month < 1 || month > 12 || hour < 0 || hour > 23 || minute < 0 || minute > 59 || second < 0 || second > 59) {
    return false;
  }

  uint8_t max_day = days_per_month[month - 1];
  if (month == 2 && Is_Leap_Year(year)) {
    max_day = 29;
  }
  return day >= 1 && day <= max_day;
}

static uint8_t Day_Of_Week(uint16_t year, uint8_t month, uint8_t day) {
  static const uint8_t offsets[] = { 0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4 };
  if (month < 3) {
    year--;
  }
  return (year + year / 4 - year / 100 + year / 400 + offsets[month - 1] + day) % 7;
}

static void Print_RTC(void) {
  PCF85063_Read_Time(&datetime);
  Serial.printf("RTC=%04u-%02u-%02u %02u:%02u:%02u\r\n",
                datetime.year, datetime.month, datetime.day,
                datetime.hour, datetime.minute, datetime.second);
}

static void Handle_RTC_Command(const char *command) {
  if (strcmp(command, "TIME?") == 0) {
    Print_RTC();
    return;
  }

  int year;
  int month;
  int day;
  int hour;
  int minute;
  int second;
  if (sscanf(command, "TIME=%d-%d-%d %d:%d:%d",
             &year, &month, &day, &hour, &minute, &second)
        != 6
      || !Is_Valid_Date_Time(year, month, day, hour, minute, second)) {
    Serial.println("Use: TIME=YYYY-MM-DD HH:MM:SS");
    return;
  }

  datetime_t new_time = { 0 };
  new_time.year = year;
  new_time.month = month;
  new_time.day = day;
  new_time.dotw = Day_Of_Week(year, month, day);
  new_time.hour = hour;
  new_time.minute = minute;
  new_time.second = second;
  PCF85063_Set_All(new_time);
  Print_RTC();
}

static void RTC_Serial_Loop(void) {
  while (Serial.available() > 0) {
    char input = Serial.read();
    if (input == '\r') {
      continue;
    }
    if (input == '\n') {
      rtc_command[rtc_command_length] = '\0';
      if (rtc_command_length > 0) {
        Handle_RTC_Command(rtc_command);
      }
      rtc_command_length = 0;
      continue;
    }
    if (rtc_command_length < sizeof(rtc_command) - 1) {
      rtc_command[rtc_command_length++] = input;
    }
  }
}

void Driver_Loop(void *parameter) {
  (void)parameter;
  while (1) {
    RTC_Serial_Loop();
    PWR_Loop();
    QMI8658_Loop();
    PCF85063_Loop();
    BAT_Get_Volts();
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
void Driver_Init() {
  PWR_Init();
  BAT_Init();
  I2C_Init();
  TCA9554PWR_Init(0x00);
  Backlight_Init();
  PCF85063_Init();
  PCF85063_Loop();
  QMI8658_Init();
}

void setup() {
  Serial.begin(115200);
  Driver_Init();
  Repulse_Storage_Init();
  Repulse_Link_Init();
  LCD_Init();
  Lvgl_Init();
  Repulse_UI_Init();
  lv_refr_now(NULL);
  Serial.println("Set RTC: TIME=YYYY-MM-DD HH:MM:SS");
  Serial.println("Read RTC: TIME?");

  xTaskCreatePinnedToCore(
    Driver_Loop,
    "Board drivers",
    2048,
    NULL,
    3,
    NULL,
    0);
}

void loop() {
  Repulse_Link_Loop();
  Lvgl_Loop();
  vTaskDelay(pdMS_TO_TICKS(5));
}
