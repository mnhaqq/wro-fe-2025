#ifdef ESC_DRIVER
   #define ESC_PIN 3
#endif

void initMotorController();
void setMotorSpeed(int spd);
void setMotorSpeeds(int motorSpeed);