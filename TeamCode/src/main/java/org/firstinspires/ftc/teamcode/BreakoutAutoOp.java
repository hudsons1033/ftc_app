package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class BreakoutAutoOp extends OpMode {

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
    public void loop() {

        //Run autonomous code
        servoA.setPosition(1);
        servoB.setPosition(1);
        motorA.setPower(1);
        motorB.setPower(1);
        try {
            wait(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        servoA.setPosition(0);
        servoB.setPosition(0);
        motorA.setPower(0);
        motorB.setPower(0);
        try {
            wait(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

    }

}
