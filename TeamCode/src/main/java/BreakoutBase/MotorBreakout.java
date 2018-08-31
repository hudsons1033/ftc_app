package BreakoutBase;

import com.qualcomm.robotcore.hardware.DcMotor;

class MotorBreakout {

    private DcMotor motor;

    public MotorBreakout(DcMotor motor) {
        this.motor = motor;
    }

    public void setDirection(DcMotor.Direction direction) {
        motor.setDirection(direction);
    }

    public void setPower(double power) {
        motor.setPower(power);
    }
}
