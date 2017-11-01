package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

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
        wheels.init(motorLeft, motorRight, motorElevator);
    }

    @Override
    public void start() {
        wheels.start();
    }


    public void loop() {
        // Gamepad sticks are inverted.
        wheels.move(-gamepad1.left_stick_y, -gamepad1.right_stick_y, -gamepad2.left_stick_y);


        telemetry.addData("Left Y", gamepad1.left_stick_y);
        telemetry.addData("Right Y", gamepad1.right_stick_y);
        telemetry.addData("Elevator Y", gamepad2.left_stick_y);
    }

    @Override
    public void stop() {
        wheels.stop();
    }
}




