package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;

public class ServoBreakout {

    private Servo servo;

    public ServoBreakout(Servo servo) {
        this.servo = servo;
    }

    public void setDirection(Servo.Direction direction) {
        servo.setDirection(direction);
    }

    public void setPosition(double position) {
        servo.setPosition(position);
    }

}
