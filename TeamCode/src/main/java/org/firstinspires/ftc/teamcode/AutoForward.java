package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name = "AutoForward", group = "Pushbot")

public class AutoForward extends OpMode {

    private DcMotor leftSide;
    private DcMotor rightSide;


    private double Speed = 0.4;
    private double secPerFt = 0.5 / Speed;
    private double straightDist = 3;
    private double straightDur = straightDist * secPerFt;

    private ElapsedTime timer = new ElapsedTime();

    private void setLeftPower(double pwr) {
        double leftPower = Range.clip(pwr, -1, 1);
        leftSide.setPower(leftPower);
    }

    private void setRightPower(double pwr) {
        double rightPower = Range.clip(pwr, -1, 1);
        rightSide.setPower(rightPower);
    }

    private void setPower(double l, double r) {
        setLeftPower(l);
        setRightPower(r);
    }

    private void move() {
        setPower(Speed, Speed);
    }

    private void setWheelMode(DcMotor.RunMode mode) {
        leftSide.setMode(mode);
        rightSide.setMode(mode);
    }

    @Override
    public void stop() {
        setPower(0, 0);
    }

    @Override
    public void start() {
        setPower(0.0, 0.0);
        setWheelMode(DcMotor.RunMode.RUN_TO_POSITION);
        timer.reset();
    }

    @Override
    public void loop() {
        move();

        telemetry.addData("Time", timer.time());

        if (timer.time() >= straightDur) {
            stop();
        }
    }

    @Override
    public void init() {
        leftSide = hardwareMap.dcMotor.get("MotorLeft");
        rightSide = hardwareMap.dcMotor.get("MotorRight");

        leftSide.setDirection(DcMotor.Direction.FORWARD);
        rightSide.setDirection(DcMotor.Direction.REVERSE);

        setWheelMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
