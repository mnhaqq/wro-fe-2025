#ifdef ESC_DRIVER
   #define ESC_PIN 5
#endif

void initMotorController();
void setMotorSpeed(int spd);
void setMotorSpeeds(int motorSpeed);