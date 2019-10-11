package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.breakout.BreakoutMotor;

public class bentenopmode extends OpMode {

    BreakoutMotor FL = new BreakoutMotor();
    BreakoutMotor FR = new BreakoutMotor();
    BreakoutMotor BL = new BreakoutMotor();
    BreakoutMotor BR = new BreakoutMotor();

    @Override
    public void init() {
        FL.set(hardwareMap.dcMotor.get("FL"));
        FR.set(hardwareMap.dcMotor.get("FR"));
        BL.set(hardwareMap.dcMotor.get("BL"));
        BR.set(hardwareMap.dcMotor.get("BR"));
        FL.setDirection(BreakoutMotor.Direction.MOTOR_F);
        FR.setDirection(BreakoutMotor.Direction.MOTOR_F);
        BL.setDirection(BreakoutMotor.Direction.MOTOR_R);
        BR.setDirection(BreakoutMotor.Direction.MOTOR_R);
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
    }

    @Override
    public void start() {
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
    }

    @Override
    public void loop() {
        FL.setPower(gamepad1.left_stick_y);
        FR.setPower(gamepad1.left_stick_y);
        BL.setPower(gamepad1.left_stick_y);
        BR.setPower(gamepad1.left_stick_y);
    }

    @Override
    public void stop() {
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
    }
}
