package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * This class is used to simplify the usage of DC Motors in the code.
 * Please use this class instead of DcMotor.
 */
class BreakoutMotor {

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
    public void setDirection(DcMotor.Direction direction) {
        motor.setDirection(direction);
    }

    //Set the power the motor turns at (0-1)
    public void setPower(double power) {
        motor.setPower(power);
    }

}
