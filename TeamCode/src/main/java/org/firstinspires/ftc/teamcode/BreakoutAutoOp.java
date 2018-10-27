package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;
import static org.firstinspires.ftc.teamcode.BreakoutMotor.Direction.MOTOR_F;
import static org.firstinspires.ftc.teamcode.BreakoutMotor.Direction.MOTOR_R;

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

    private static final float mmPerInch        = 25.4f;
    private static final float mmFTCFieldWidth  = (12*6) * mmPerInch;
    private static final float mmTargetHeight   = (6) * mmPerInch;

    private List<VuforiaTrackable> allTrackables = new ArrayList<>();

    private BreakoutMotor motorA = new BreakoutMotor();
    private BreakoutMotor motorB = new BreakoutMotor();
    private BreakoutMotor motorC = new BreakoutMotor();
    private BreakoutMotor motorD = new BreakoutMotor();

    private BreakoutREVGyro gyroA = new BreakoutREVGyro();

    private ElapsedTime timer = new ElapsedTime();
    private ElapsedTime timer2 = new ElapsedTime();

    private double accumulation_of_error;
    private double last_error;

    private double pid() {
        double delta_time = timer2.now(TimeUnit.MILLISECONDS);
        double error = 0 - gyroA.getOrient().firstAngle;
        accumulation_of_error += error * delta_time;
        double derivative_of_error = (error - last_error) / delta_time;
        last_error = error;
        double out = (error * 0.1) + (accumulation_of_error * 0) + (derivative_of_error * 0);
        telemetry.addData("delta_time", delta_time);
        telemetry.addData("error", error);
        telemetry.addData("accumulation of error", accumulation_of_error);
        telemetry.addData("derivative of error", derivative_of_error);
        telemetry.addData("last error", last_error);
        telemetry.addData("out", out);
        return out;
    }

    @Override
    public void init() {

        timer.reset();
        //Broken out motor class
        motorA.set(hardwareMap.dcMotor.get("motorA"));
        motorB.set(hardwareMap.dcMotor.get("motorB"));
        motorC.set(hardwareMap.dcMotor.get("motorC"));
        motorD.set(hardwareMap.dcMotor.get("motorD"));
        motorA.setDirection(MOTOR_F);
        motorB.setDirection(MOTOR_F);
        motorC.setDirection(MOTOR_F);
        motorD.setDirection(MOTOR_F);
        motorA.setPower(0);
        motorB.setPower(0);
        motorC.setPower(0);
        motorD.setPower(0);

        //Broken out Gyro class
        gyroA.set(hardwareMap.get(gyroA.IMU, "gyroA"));
        telemetry.addLine("Calibrating: DO NOT MOVE!");
        gyroA.calibrate();
        telemetry.clearAll();

    }

    @Override
    public void start() {
        motorA.setPower(0);
        motorB.setPower(0);
        motorC.setPower(0);
        motorD.setPower(0);
    }

    private void blueRover() {
        timer2.reset();
        motorB.setPower(0.5);
        motorC.setPower(0);
        motorD.setPower(0);
        motorA.setPower(Math.max(0, Math.min(1, pid()+0.5)));
        telemetry.update();
        //motorA.setPower(0);
        //motorB.setPower(0);
    }

    private void redFootprint() {
        motorB.setPower(1);
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        motorB.setPower(0);
    }

    private void frontCraters() {
        motorC.setPower(1);
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        motorC.setPower(0);
    }

    private void backSpace() {
        motorD.setPower(1);
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        motorD.setPower(0);
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

            OpenGLMatrix blueRoverLocationOnField = OpenGLMatrix
                    .translation(0, mmFTCFieldWidth, mmTargetHeight)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0));
            blueRover.setLocation(blueRoverLocationOnField);

            OpenGLMatrix redFootprintLocationOnField = OpenGLMatrix
                    .translation(0, -mmFTCFieldWidth, mmTargetHeight)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180));
            redFootprint.setLocation(redFootprintLocationOnField);

            OpenGLMatrix frontCratersLocationOnField = OpenGLMatrix
                    .translation(-mmFTCFieldWidth, 0, mmTargetHeight)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90));
            frontCraters.setLocation(frontCratersLocationOnField);

            OpenGLMatrix backSpaceLocationOnField = OpenGLMatrix
                    .translation(mmFTCFieldWidth, 0, mmTargetHeight)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90));
            backSpace.setLocation(backSpaceLocationOnField);

            final int CAMERA_FORWARD_DISPLACEMENT  = 110;   // eg: Camera is 110 mm in front of robot center
            final int CAMERA_VERTICAL_DISPLACEMENT = 200;   // eg: Camera is 200 mm above ground
            final int CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

            OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                    .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES,
                            CAMERA_CHOICE == FRONT ? 90 : -90, 0, 0));

            /*  Let all the trackable listeners know where the phone is.  */
            for (VuforiaTrackable trackable : allTrackables)
            {
                ((VuforiaTrackableDefaultListener)trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
            }
        }

        if (!found) {
            motorA.setPower(1);
            motorB.setPower(1);
            motorC.setPower(1);
            motorD.setPower(1);
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                    this.trackableName = trackable.getName();
                    telemetry.addData("Visible Target", trackable.getName());
                    found = true;
                    timer2.reset();
                    break;
                }
            }
        }

//        switch (trackableName) {
//            case "Blue-Rover":
//                blueRover();
//            case "Red-Footprint":
//                redFootprint();
//                stop();
//                break;
//            case "Front-Craters":
//                frontCraters();
//                stop();
//                break;
//            case "Back-Space":
//                backSpace();
//                stop();
//                break;
//            case "":
//                break;
//        }
        blueRover();
    }

    @Override
    public void stop() {
        motorA.setPower(0);
        motorB.setPower(0);
        motorC.setPower(0);
        motorD.setPower(0);
    }
}
