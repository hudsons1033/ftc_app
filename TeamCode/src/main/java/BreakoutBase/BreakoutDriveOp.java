package BreakoutBase;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Drive", group = "Pushbot")

public class BreakoutDriveOp extends OpMode {

    private DcMotor motorA;
    private DcMotor motorB;

    @Override
    public void init() {
        DcMotor motorA = hardwareMap.dcMotor.get("motorA");
        DcMotor motorB = hardwareMap.dcMotor.get("motorB");
    }

    @Override
    public void start() {
        motorA.setPower(0);
        motorB.setPower(0);
    }

    @Override
    public void loop() {

        //Gamepad 1
        float leftStick1x = gamepad1.left_stick_x;
        float leftStick1y = gamepad1.left_stick_y;
        float rightStick1x = gamepad1.right_stick_x;
        float rightStick1y = gamepad1.right_stick_y;
        //Gamepad 2
        float leftStick2x = gamepad2.left_stick_x;
        float leftStick2y = gamepad2.left_stick_y;
        float rightStick2x = gamepad2.right_stick_x;
        float rightStick2y = gamepad2.right_stick_y;

        //Move the motors
        motorA.setPower(-leftStick1y);
        motorB.setPower(-rightStick1y);

        //Telemetry
        telemetry.addData("Left Stick X 1", leftStick1x);
        telemetry.addData("Left Stick Y 1", leftStick1y);
        telemetry.addData("Right Stick X 1", rightStick1x);
        telemetry.addData("Right Stick Y 1", rightStick1y);
        telemetry.addData("Left Stick X 2", leftStick2x);
        telemetry.addData("Left Stick Y 2", leftStick2y);
        telemetry.addData("Right Stick X 2", rightStick2x);
        telemetry.addData("Right Stick Y 2", rightStick2y);

    }

    @Override
    public void stop() {
        motorA.setPower(0);
        motorB.setPower(0);
    }

}
