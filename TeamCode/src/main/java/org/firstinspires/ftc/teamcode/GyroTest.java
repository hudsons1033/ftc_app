package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "GyroTest", group = "pushbot")

public class GyroTest extends OpMode {

    BreakoutREVGyro gyro = new BreakoutREVGyro();

    @Override
    public void init() {
        gyro.set(hardwareMap.get(gyro.IMU, "gyroA"));
        telemetry.addLine("Calibrating: DO NOT MOVE!");
        gyro.calibrate();
        telemetry.clearAll();
    }

    @Override
    public void loop() {
        telemetry.addData("Gyro Heading", gyro.getPos());
        telemetry.addData("Gyro Orientation", gyro.getOrient());
        telemetry.addData("Gyro Velocity", gyro.getVel());
        telemetry.addData("Gyro Acceleration", gyro.getAccel());
    }

}
