package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;

/*
This class is used to simplify the usage of Servos in the code.
Please use this class instead of the Servo class.
 */

public class BreakoutServo {

    //Define servo as a Servo object
    private Servo servo;

    //Retrieve servo var
    public Servo get() {
        return servo;
    }

    //Set hardware map data of servo
    public void set(Servo servo) {
        this.servo = servo;
    }

    //Set the direction the servo rotates in (FORWARD or REVERSE)
    public void setDirection(Servo.Direction direction) {
        servo.setDirection(direction);
    }

    //Set the position the servo is in (0-1)
    public void setPosition(double position) {
        servo.setPosition(position);
    }

}
