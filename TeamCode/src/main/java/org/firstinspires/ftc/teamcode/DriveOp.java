package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Drive", group = "Pushbot")

public class DriveOp extends OpMode {

    private Wheels wheels;

    public DriveOp() {
        wheels = new Wheels();
    }

    @Override
    public void init() {
        DcMotor motorLeft = hardwareMap.dcMotor.get("MotorLeft");
        DcMotor motorRight = hardwareMap.dcMotor.get("MotorRight");
        DcMotor motorElevator = hardwareMap.dcMotor.get("ElevatorMotor");
        Servo servoLeft = hardwareMap.servo.get("ServoLeft");
        Servo servoRight = hardwareMap.servo.get("ServoRight");
        wheels.init(motorLeft, motorRight, motorElevator, servoLeft, servoRight);
    }

    @Override
    public void start() {
        wheels.start();
    }


    public void loop() {
        // Gamepad sticks are inverted.
        wheels.move(-gamepad1.left_stick_y, -gamepad1.right_stick_y, -gamepad2.left_stick_y, gamepad2.a);

        telemetry.addData("Left Y", gamepad1.left_stick_y);
        telemetry.addData("Right Y", gamepad1.right_stick_y);
        telemetry.addData("Elevator Y", gamepad2.left_stick_y);
        telemetry.addData("Button A", gamepad2.a);
    }

    @Override
    public void stop() {
        wheels.stop();
    }
}




