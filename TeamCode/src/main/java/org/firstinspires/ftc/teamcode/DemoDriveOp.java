package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.BreakoutMotor.Direction.MOTOR_R;

/**
 * This class is used for the main game to drive the robot using the controllers.
 **/
@TeleOp(name = "Demo", group = "Pushbot")

public class DemoDriveOp extends OpMode {
    //Motor objects
    private BreakoutMotor motorLeft = new BreakoutMotor();
    private BreakoutMotor motorRight = new BreakoutMotor();

    @Override
    public void init() {
        //Broken out motor class
        //Set hardwaremaps for left and right motors
        motorLeft.set(hardwareMap.dcMotor.get("motorLeft"));
        motorRight.set(hardwareMap.dcMotor.get("motorRight"));
        //Set directions for left and right motors
        motorLeft.setDirection(MOTOR_R);
        motorRight.setDirection(MOTOR_R);
        //Start with 0 power for both motors
        motorLeft.setPower(0);
        motorRight.setPower(0);

        //Clear telemetry
        telemetry.clearAll();
    }

    @Override
    public void start() {
        //Motor start
        motorLeft.setPower(0);
        motorRight.setPower(0);
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

        //Move the motors//
        if (rightStick1x > 0) {
            motorLeft.setPower(-leftStick1y);
            motorRight.setPower(-leftStick1y-rightStick1x);
        } else if (rightStick1x < 0) {
            motorLeft.setPower(-leftStick1y+rightStick1x);
            motorRight.setPower(-leftStick1y);
        } else {
            motorLeft.setPower(0);
            motorRight.setPower(0);
        }

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
        //Motor stop
        motorLeft.setPower(0);
        motorRight.setPower(0);
    }
}
