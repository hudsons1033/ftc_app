package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "GyroTest", group = "pushbot")

public class GyroTest extends OpMode {

    private BreakoutREVGyro gyro = new BreakoutREVGyro();
    private BreakoutMotor motorLeft = new BreakoutMotor();
    private BreakoutMotor motorRight = new BreakoutMotor();

    @Override
    public void init() {
        gyro.set(hardwareMap.get(gyro.IMU, "gyroA"));
        telemetry.addLine("Calibrating: DO NOT MOVE!");
        gyro.calibrate();
        telemetry.clearAll();
    }

    @Override
    public void loop() {
        double x = Math.abs(gyro.getOrient().firstAngle);
        double plus5 = x + 0.5;
        double minus5 = x - 0.5;

        if (plus5 > 1) {
            plus5 = 1;
        }
        if (minus5 < -1) {
            minus5 = -1;
        }

        if (gyro.getOrient().firstAngle > 0) {
            motorLeft.setPower(plus5);
            motorRight.setPower(minus5);
        } else {
            motorLeft.setPower(minus5);
            motorRight.setPower(plus5);
        }

        telemetry.addData("Gyro Heading", gyro.getPos());
        telemetry.addData("Gyro Orientation", gyro.getOrient());
        telemetry.addData("Gyro Velocity", gyro.getVel());
        telemetry.addData("Gyro Acceleration", gyro.getAccel());
    }

}
