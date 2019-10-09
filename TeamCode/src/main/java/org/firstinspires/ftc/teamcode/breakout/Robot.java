package org.firstinspires.ftc.teamcode.breakout;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static org.firstinspires.ftc.teamcode.breakout.BreakoutMotor.Direction.MOTOR_F;
import static org.firstinspires.ftc.teamcode.breakout.BreakoutMotor.Direction.MOTOR_R;

public class Robot {

    public enum Motor {
        FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT
    }

    private BreakoutMotor frontLeft = new BreakoutMotor();
    private BreakoutMotor frontRight = new BreakoutMotor();
    private BreakoutMotor backLeft = new BreakoutMotor();
    private BreakoutMotor backRight = new BreakoutMotor();

    private BreakoutREVGyro gyro = new BreakoutREVGyro();
    private Orientation lastAngles = new Orientation();
    private double globalAngle = 0;
    private double correction = 0;

    private HardwareMap hardwareMap;
    private ElapsedTime period = new ElapsedTime();
    private Telemetry telemetry;

    /* Constructor */
    public Robot(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public void setPower(Motor motor, float power) {
        switch (motor) {
            case FRONT_LEFT:
                frontLeft.setPower(power);
                break;
            case FRONT_RIGHT:
                frontRight.setPower(power);
                break;
            case BACK_LEFT:
                backLeft.setPower(power);
                break;
            case BACK_RIGHT:
                backRight.setPower(power);
                break;
        }
    }

    public void setRunMode(Motor motor, DcMotor.RunMode mode) {
        switch (motor) {
            case FRONT_LEFT:
                frontLeft.setMotorMode(mode);
                break;
            case FRONT_RIGHT:
                frontRight.setMotorMode(mode);
                break;
            case BACK_LEFT:
                backLeft.setMotorMode(mode);
                break;
            case BACK_RIGHT:
                backRight.setMotorMode(mode);
                break;
        }
    }

    public void setTargetPosition(Motor motor, int pos) {
        switch (motor) {
            case FRONT_LEFT:
                frontLeft.setTargetPosition(pos);
                break;
            case FRONT_RIGHT:
                frontRight.setTargetPosition(pos);
                break;
            case BACK_LEFT:
                backLeft.setTargetPosition(pos);
                break;
            case BACK_RIGHT:
                backRight.setTargetPosition(pos);
                break;
        }
    }

    public boolean isBusy(Motor motor) {
        switch (motor) {
            case FRONT_LEFT:
                return frontLeft.isBusy();
            case FRONT_RIGHT:
                return frontRight.isBusy();
            case BACK_LEFT:
                return backLeft.isBusy();
            case BACK_RIGHT:
                return backRight.isBusy();
            default:
                return false;
        }
    }

    public int getCurrentPosition(Motor motor) {
        switch (motor) {
            case FRONT_LEFT:
                return frontLeft.getCurrentPosition();
            case FRONT_RIGHT:
                return frontRight.getCurrentPosition();
            case BACK_LEFT:
                return backLeft.getCurrentPosition();
            case BACK_RIGHT:
                return backRight.getCurrentPosition();
            default:
                return 8008135;
        }
    }

    public Orientation getAngularOrientation() {
        return gyro.getOrient(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);
    }

    public void resetAngle() {
        lastAngles = gyro.getOrient(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    private double getAngle() {
        Orientation current = getAngularOrientation();

        double deltaAngle = current.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180) {
            deltaAngle += 360;
        } else if (deltaAngle > 180) {
            deltaAngle -= 360;
        }

        globalAngle += deltaAngle;
        lastAngles = current;

        return globalAngle;
    }

    public double checkDirection() {
        //TODO: find a good c value
        double correction, angle, c = -0.01;
        angle = getAngle();

        if (angle == 0) {
            correction = 0;
        } else if (angle < 360 || angle > -360){
            correction = -angle;
        } else {
            resetAngle();
            correction = 0;
        }

        correction *= c;
        telemetry.addData("Correction", correction);
        telemetry.addData("Angle", angle);
        return correction;
    }

    public Orientation getLastAngles() {
        return lastAngles;
    }

    public double getGlobalAngle() {
        return globalAngle;
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap hardwareMap) {
        // Save reference to Hardware map
        this.hardwareMap = hardwareMap;

        // Gyro init
        gyro.set(hardwareMap.get(gyro.IMU, "gyroA"));
        telemetry.addLine("Calibrating: DO NOT MOVE!");
        telemetry.update();
        gyro.calibrate();
        telemetry.clearAll();

        // Define and Initialize Motors
        frontLeft.set(hardwareMap.dcMotor.get("frontLeft"));
        frontRight.set(hardwareMap.dcMotor.get("frontRight"));
        backLeft.set(hardwareMap.dcMotor.get("backLeft"));
        backRight.set(hardwareMap.dcMotor.get("backRight"));

        //Set directions for left and right motors
        //F = Clockwise while looking at axle
        //R = Counter clockwise while looking at axle
        frontLeft.setDirection(MOTOR_R);
        frontRight.setDirection(MOTOR_F);
        backLeft.setDirection(MOTOR_R);
        backRight.setDirection(MOTOR_F);

        // Set all motors to zero power
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        frontLeft.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Define and initialize ALL installed servos.
    }
}
