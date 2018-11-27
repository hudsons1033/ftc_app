package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.teamcode.BreakoutMotor.Direction.MOTOR_R;
import static org.firstinspires.ftc.teamcode.BreakoutMotor.Direction.MOTOR_F;
import static org.firstinspires.ftc.teamcode.BreakoutServo.Direction.SERVO_F;
import static org.firstinspires.ftc.teamcode.BreakoutServo.Direction.SERVO_R;

/**
 * This class is used for the Autonomous segment of the game. No controllers are to be used.
 **/
@Autonomous(name = "Autonomous", group = "pushbot")

public class BreakoutAutoOp extends OpMode {

    private static final String VUFORIA_KEY = "AT5/naD/////AAABmaUvi2GyUUirkbnkPyo71cQmkhAaUVAKxkF96tVK5PueT2cxswHD7dT7ZBvwEBS4oPEDsajO7QdUFF0VtshBOnfi5AFTlL5pfgmMQhr2ceo9xTa+pyw71Bvym3qvmv5utemN+yU5b8eQfejOyfCh/EGpSvclNB9JAa5125hevdhFoYOT7BU6/Xx1wBAith2m/RPesA1tqqBfjWC4KfBonwCtLa0+erPLNnGt0PJiM0CIxyFTo4u9V4AP1TQm/VooDJ8+yXN7ZiIimPG42+0ahoYOdBK6Tc3jX2idV4YCSPBv8Gz6Ut9fu9VLMiYkDUeGZ9prKeehGh7NtttajEe6Dw/h3ww1sCoKOTfJvK0dWi3h";

    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;

    private boolean found = false;
    private boolean firstRun = true;
    private String trackableName = "";
    double i = 0;

    private List<VuforiaTrackable> allTrackables = new ArrayList<>();
    private List<BreakoutMotor> allMotors = new ArrayList<>();
    private List<BreakoutServo> allServos = new ArrayList<>();

    private BreakoutMotor motorLeft = new BreakoutMotor();
    private BreakoutMotor motorRight = new BreakoutMotor();
    private BreakoutMotor motorSweeper = new BreakoutMotor();
    private BreakoutMotor motorSweeperArm = new BreakoutMotor();
    private BreakoutMotor motorHorizontal = new BreakoutMotor();
    private BreakoutMotor motorHorizontalBack = new BreakoutMotor();

    private BreakoutREVGyro gyroA = new BreakoutREVGyro();

    private ElapsedTime startTime = new ElapsedTime();

    private double integral;
    private double last_error;

    private void allSet(List<BreakoutMotor> motors, int job) {
        for (BreakoutMotor motor : motors) {
            setThing(motor, job);
        }
    }

    private void setThing(BreakoutMotor motor, int job) {
        if (job == 1) {
            motor.setPower(0);
        } else if (job == 2) {
            motor.setDirection(MOTOR_F);
        }
    }

//    double Kp = 0.5;
//    double Ki = 0.0;
//    double Kd = 0.0;
//
//    private double pid(double cenLinAng) {
//        double delta_time = startTime.now(TimeUnit.MILLISECONDS);
//        double error = cenLinAng - gyroA.getOrient().firstAngle;
//        integral += error * delta_time;
//        double derivative = (error - last_error) / delta_time;
//        last_error = error;
//        double var = (error * Kp) + (integral * Ki) + (derivative * Kd);
//        double out = 1-var;
//        telemetry.addData("angle", gyroA.getOrient().firstAngle);
//        telemetry.addData("delta_time", delta_time);
//        telemetry.addData("error", error);
//        telemetry.addData("accumulation of error", integral);
//        telemetry.addData("derivative of error", derivative);
//        telemetry.addData("last error", last_error);
//        telemetry.addData("out", out);
//        return out;
//    }
//    private double pid2(double cenLinAng) {
//        double delta_time = startTime.now(TimeUnit.MILLISECONDS);
//        double error = cenLinAng + gyroA.getOrient().firstAngle;
//        integral += error * delta_time;
//        double derivative = (error - last_error) / delta_time;
//        last_error = error;
//        double var = (error * Kp) + (integral * Ki) + (derivative * Kd);
//        double out = 1-var;
//        telemetry.addData("angle", gyroA.getOrient().firstAngle);
//        telemetry.addData("delta_time", delta_time);
//        telemetry.addData("error", error);
//        telemetry.addData("accumulation of error", integral);
//        telemetry.addData("derivative of error", derivative);
//        telemetry.addData("last error", last_error);
//        telemetry.addData("out", out);
//        return out;
//    }
//
//    private void blueRover() {
//        double centerAng = 0;
//        if(gyroA.getOrient().firstAngle <= centerAng) {
//            motorLeft.setPower(Math.max(0, Math.min(1, pid(centerAng)))+0.5);
//            motorRight.setPower(0.5);
//        } else {
//            motorLeft.setPower(0.5);
//            motorRight.setPower(Math.max(0, Math.min(1, pid2(centerAng)))+0.5);
//        }
//        telemetry.update();
//    }

    private void blueRover() {
        while(startTime.milliseconds() < 1700) {
            motorLeft.setPower(1);
            motorRight.setPower(1);
        }
        motorLeft.setPower(0);
        motorRight.setPower(0);
        while(startTime.milliseconds() < 2000) {
            motorSweeperArm.setPower(1);
        }
        motorSweeperArm.setPower(0);
        while(startTime.milliseconds() < 2500) {
            motorSweeper.setPower(-1);
        }
        motorSweeper.setPower(0);

    }

    private void redFootprint() {
        while(startTime.milliseconds() < 1700) {
            motorLeft.setPower(1);
            motorRight.setPower(1);
        }
        while(startTime.now(TimeUnit.MILLISECONDS) < 2700) {
            motorLeft.setPower(0);
            motorRight.setPower(0);
            motorSweeperArm.setPower(1);
            motorSweeper.setPower(1);
        }
        stop();
    }

    private void frontCraters() {
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    private void backSpace() {
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    @Override
    public void init() {

        //Broken out motorSweeper class
        motorLeft.set(hardwareMap.dcMotor.get("motorLeft"));
        motorRight.set(hardwareMap.dcMotor.get("motorRight"));
        motorSweeper.set(hardwareMap.dcMotor.get("motorSweeper"));
        motorSweeperArm.set(hardwareMap.dcMotor.get("motorSweeperArm"));
        motorHorizontal.set(hardwareMap.dcMotor.get("motorHorizontal"));
        motorHorizontalBack.set(hardwareMap.dcMotor.get("motorHorizontalBack"));
        motorLeft.setDirection(MOTOR_R);
        motorRight.setDirection(MOTOR_R);
        motorSweeper.setDirection(MOTOR_F);
        motorSweeperArm.setDirection(MOTOR_F);
        motorHorizontal.setDirection(MOTOR_F); //?
        motorHorizontalBack.setDirection(MOTOR_R); //?
        allMotors.add(motorLeft); allMotors.add(motorRight); allMotors.add(motorSweeper); allMotors.add(motorSweeperArm); allMotors.add(motorHorizontal); allMotors.add(motorHorizontalBack);
        allSet(allMotors, 1);

        //Broken out Gyro class
        gyroA.set(hardwareMap.get(gyroA.IMU, "gyroA"));
        telemetry.addLine("Calibrating: DO NOT MOVE!");
        telemetry.update();
        gyroA.calibrate();
        telemetry.clearAll();
        telemetry.update();
    }

    @Override
    public void start() {
        allSet(allMotors, 1);
    }


    @Override
    public void loop() {

        if (firstRun) {
            firstRun = false;

            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

            parameters.vuforiaLicenseKey = VUFORIA_KEY;
            parameters.cameraDirection = CAMERA_CHOICE;

            VuforiaLocalizer vuforia = ClassFactory.getInstance().createVuforia(parameters);
            VuforiaTrackables targetsRoverRuckus = vuforia.loadTrackablesFromAsset("RoverRuckus");
            VuforiaTrackable blueRover = targetsRoverRuckus.get(0);
            blueRover.setName("Blue-Rover");
            VuforiaTrackable redFootprint = targetsRoverRuckus.get(1);
            redFootprint.setName("Red-Footprint");
            VuforiaTrackable frontCraters = targetsRoverRuckus.get(2);
            frontCraters.setName("Front-Craters");
            VuforiaTrackable backSpace = targetsRoverRuckus.get(3);
            backSpace.setName("Back-Space");

            allTrackables.addAll(targetsRoverRuckus);

            targetsRoverRuckus.activate();
            startTime.reset();
        }

        if (!found) {
            motorLeft.setPower(1);
            motorRight.setPower(-1);
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                    this.trackableName = trackable.getName();
                    telemetry.addData("Visible Target", trackable.getName());
                    found = true;
                    startTime.reset();
                    break;
                }
            }
        }

        switch (trackableName) {
            case "Blue-Rover":
                blueRover();
                break;
            case "Red-Footprint":
                redFootprint();
                break;
            case "Front-Craters":
                frontCraters();
                break;
            case "Back-Space":
                backSpace();
                break;
            case "":
                startTime.reset();
                break;
        }

    }

    @Override
    public void stop() {
        allSet(allMotors, 1);
    }
}
