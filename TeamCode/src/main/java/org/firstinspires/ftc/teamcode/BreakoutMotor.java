package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * This class is used to simplify the usage of DC Motors in the code.
 * Please use this class instead of DcMotor.
 */
class BreakoutMotor {

    //Enum for direction
    enum Direction {
        MOTOR_FORWARD, MOTOR_REVERSE
    }

    //Define motor as a DcMotor
    private DcMotor motor;

    //Retrieve motor var
    public DcMotor get() {
        return motor;
    }

    //Set hardware map data of motor
    public void set(DcMotor motor) {
        this.motor = motor;
    }

    //Set the direction the motor turns in (FORWARD or REVERSE)
    public void setDirection(Direction direction) {

        if (direction == Direction.MOTOR_FORWARD) {
            motor.setDirection(DcMotorSimple.Direction.FORWARD);
        } else {
            motor.setDirection(DcMotorSimple.Direction.REVERSE);
        }

    }

    //Set the power the motor turns at (0-1)
    public void setPower(double power) {
        motor.setPower(power);
    }

}
