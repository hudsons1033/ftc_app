package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "GyroTest", group = "pushbot")

public class GyroTest extends OpMode {

    BreakoutGyro gyro = new BreakoutGyro();

    @Override
    public void init() {
        gyro.set(hardwareMap.gyroSensor.get("gyroA"));
        gyro.calibrate();
        while (gyro.gyroSensor.isCalibrating()) {
            telemetry.addData("Calibrating", "");
        }
        telemetry.clear();
    }

    @Override
    public void loop() {
        telemetry.addData("Gyro Heading", gyro.getHeading());
        telemetry.addData("Gyro Angular Velocity XYZ", gyro.getAngleVelocityXYZ());
        telemetry.addData("Gyro Raw XYZ", gyro.getRawXYZ());
        telemetry.addData("Gyro Orientation XYZ", gyro.getOrientationXYZ());
    }
}
