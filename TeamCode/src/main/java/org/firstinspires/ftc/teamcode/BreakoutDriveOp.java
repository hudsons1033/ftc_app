package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Drive", group = "Pushbot")

public class BreakoutDriveOp extends OpMode {

    //Direction Constants
    private DcMotor.Direction MOTOR_FORWARD = DcMotor.Direction.FORWARD;
    private Servo.Direction SERVO_FORWARD = Servo.Direction.FORWARD;
    private DcMotor.Direction MOTOR_REVERSE = DcMotor.Direction.REVERSE;
    private Servo.Direction SERVO_REVERSE = Servo.Direction.REVERSE;

    //Servo Breakout code definition
    private BreakoutServo servoA;
    private BreakoutServo servoB;

    //Motor Breakout code definition
    private BreakoutMotor motorA;
    private BreakoutMotor motorB;

    //Gyro Breakout code definition
    private BreakoutGyro gyroA;

    private ElapsedTime timer = new ElapsedTime();

    @Override
    public void init() {

        //Broken out servo class
        servoA.set(hardwareMap.servo.get("servoA"));
        servoB.set(hardwareMap.servo.get("servoB"));
        servoA.setDirection(SERVO_REVERSE);
        servoB.setDirection(SERVO_FORWARD);
        servoA.setPosition(0);
        servoB.setPosition(0);

        //Broken out motor class
        motorA.set(hardwareMap.dcMotor.get("motorA"));
        motorB.set(hardwareMap.dcMotor.get("motorB"));
        motorA.setDirection(MOTOR_FORWARD);
        motorB.setDirection(MOTOR_REVERSE);
        motorA.setPower(0);
        motorB.setPower(0);

        //Broken out Gyro class
        gyroA.set(hardwareMap.gyroSensor.get("gyroA"));
        telemetry.addLine("Calibrating: DO NOT MOVE!");
        gyroA.calibrate();
        timer.reset();
        while (gyroA.gyroSensor.isCalibrating()) {
            telemetry.addData("Calibrating: ", Math.round(timer.seconds()) + " seconds");
            telemetry.update();
            try {
                wait(50);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

        }

    }

    @Override
    public void start() {

        //Motor start
        motorA.setPower(0);
        motorB.setPower(0);

        //Servo start
        servoA.setPosition(0);
        servoB.setPosition(0);

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

        //Set the servos
        servoA.setPosition(leftStick2y);
        servoB.setPosition(rightStick1y);

        //Telemetry
        telemetry.addData("Left Stick X 1", leftStick1x);
        telemetry.addData("Left Stick Y 1", leftStick1y);
        telemetry.addData("Right Stick X 1", rightStick1x);
        telemetry.addData("Right Stick Y 1", rightStick1y);
        telemetry.addData("Left Stick X 2", leftStick2x);
        telemetry.addData("Left Stick Y 2", leftStick2y);
        telemetry.addData("Right Stick X 2", rightStick2x);
        telemetry.addData("Right Stick Y 2", rightStick2y);
        telemetry.addData("Gyro Heading", gyroA.getHeading());

    }

    @Override
    public void stop() {

        //Motor stop
        motorA.setPower(0);
        motorB.setPower(0);

        //Servo stop
        servoA.setPosition(0);
        servoB.setPosition(0);

    }

}
