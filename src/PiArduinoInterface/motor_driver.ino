#ifdef USE_BASE

#ifdef ESC_DRIVER
    Servo esc;
    void initMotorController() {
        esc.attach(ESC_PIN);
        esc.writeMicroseconds(1500);
    }

    void setMotorSpeed(int spd) {
        esc.writeMicroseconds(1500 + spd);
    }

    void setMotorSpeeds(int motorSpeed) {
        setMotorSpeed(motorSpeed);
    }
#else
    #error A motor driver must be selected
#endif

#endif