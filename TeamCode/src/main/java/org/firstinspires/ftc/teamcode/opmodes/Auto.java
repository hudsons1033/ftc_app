package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.breakout.Robot;
import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.field.FieldObject;
import org.firstinspires.ftc.teamcode.matrix.Matrix;
import org.firstinspires.ftc.teamcode.matrix.MatrixHandler;
import org.firstinspires.ftc.teamcode.mecanum.Mecanum;

import java.util.ArrayList;

@Autonomous(name = "Auto", group = "Pushbot")

public class Auto extends LinearOpMode {

    /* Declare OpMode members. */
    Robot robot = new Robot(telemetry);   // Use a Pushbot's hardware
    FieldObject robotObject;
    Mecanum drive = new Mecanum(robot, telemetry);
    private ElapsedTime runtime = new ElapsedTime();
    private Field field = new Field();

    private static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    private static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    private static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    private static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                              (WHEEL_DIAMETER_INCHES * 3.1415);
    private static final float     DRIVE_SPEED              = 0.6f;
    private static final float     TURN_SPEED               = 0.5f;

    private ArrayList<FieldObject> fieldObjects;

    @Override
    public void runOpMode() {

        robotObject = new FieldObject(0,0, 18, 18, "robot");

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.setRunMode(Robot.Motor.FRONT_LEFT, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setRunMode(Robot.Motor.FRONT_RIGHT, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setRunMode(Robot.Motor.BACK_LEFT, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setRunMode(Robot.Motor.BACK_RIGHT, DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.setRunMode(Robot.Motor.FRONT_LEFT, DcMotor.RunMode.RUN_USING_ENCODER);
        robot.setRunMode(Robot.Motor.FRONT_RIGHT, DcMotor.RunMode.RUN_USING_ENCODER);
        robot.setRunMode(Robot.Motor.BACK_LEFT, DcMotor.RunMode.RUN_USING_ENCODER);
        robot.setRunMode(Robot.Motor.BACK_RIGHT, DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d : %7d : %7d : %7d",
                robot.getCurrentPosition(Robot.Motor.FRONT_LEFT),
                robot.getCurrentPosition(Robot.Motor.FRONT_RIGHT),
                robot.getCurrentPosition(Robot.Motor.BACK_LEFT),
                robot.getCurrentPosition(Robot.Motor.BACK_RIGHT));
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        // Go straight for 48 inches with 5 sec timeout
        // encoderDrive(0,  DRIVE_SPEED,  48,  48, 48, 48, 5.0, false);
        // Go backwards for 48 inches with 5 sec timeout
        // encoderDrive(0, DRIVE_SPEED, -48, -48, -48, -48, 5.0, false);
        // Turn left 12 inches with 4 sec timeout
        // encoderDrive(TURN_SPEED, 0, -12, 12, -12, 12, 4.0, true);
        // Turn right 12 inches with 4 sec timeout
        // encoderDrive(0, TURN_SPEED, 12, -12, 12, -12, 4.0, true);
        // Move left 24 inches with 4 sec timeout
        // encoderDrive(DRIVE_SPEED, 0, -24, 24, 24, -24, 4.0, false);
        // Move right 24 inches with 4 sec timeout
        // encoderDrive(-DRIVE_SPEED, 0, 24, -24, -24, 24, 4.0, false);

//        Manipulator here
//        sleep(1000);     // pause for servos to move

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(Robot robot,
                             FieldObject robotObject, FieldObject destination,
                             float xOffset, float yOffset, float rotationDegrees,
                             double timeoutS, boolean rotate) {
        int frontLeftTarget;
        int frontRightTarget;
        int backLeftTarget;
        int backRightTarget;

        float destL = destination.getLength();
        float destW = destination.getWidth();
        float destX = destination.getX();
        float destY = destination.getY();
        float[] destTopLeft = {destX, destY + destL};
        float[] destTopRight = {destX + destW, destY + destL};
        float[] destBottomLeft = {destX, destY};
        float[] destBottomRight = {destX + destW, destY};

        float robotL = robotObject.getLength();
        float robotW = robotObject.getWidth();
        float robotX = robotObject.getX();
        float robotY = robotObject.getY();
        float[] robotTopLeft = {robotX, robotY + robotL};
        float[] robotTopRight = {robotX + robotW, robotY + robotL};
        float[] robotBottomLeft = {robotX, robotY};
        float[] robotBottomRight = {robotX + robotW, robotY};

        boolean robotIsLeftOfDest = false;
        boolean robotIsAboveDest = false;
        if (!((robotTopLeft[0]+robotTopRight[0])/2 < (destTopLeft[0]+destTopRight[0])/2)) {
            robotIsLeftOfDest = true;
        }
        if (!((robotTopLeft[1]+robotBottomLeft[1])/2 < (destTopLeft[1]+destBottomLeft[1])/2)) {
            robotIsAboveDest = true;
        }

        float xInches;
        float yInches;
        if (robotIsAboveDest) {
            yInches = -(robotBottomRight[1] - destTopRight[1]) + yOffset;
        } else {
            yInches = destBottomRight[1] - robotTopRight[1] - yOffset;
        }
        if (robotIsLeftOfDest) {
            xInches = destBottomLeft[0] - robotBottomRight[0] + xOffset;
        } else {
            xInches = -(robotBottomLeft[0] - destBottomRight[0]) - xOffset;
        }
        float zInches = (float)(((rotationDegrees*Math.PI)/180)*25.4558);

        Matrix xMatrix = new Matrix(2, 2);
        float[] driveX = {1, -1, -1, 1};
        xMatrix.setValues(driveX);
        Matrix yMatrix = new Matrix(2, 2);
        float[] driveY = {1, 1, 1, 1};
        yMatrix.setValues(driveY);

        xMatrix.scalar(xInches);
        yMatrix.scalar(yInches);
        MatrixHandler handler = new MatrixHandler(xMatrix, yMatrix);
        Matrix fin = handler.addMatrices();

        if (rotationDegrees != 0) {
            Matrix zMatrix = new Matrix(2, 2);
            float[] driveZ = {1, -1, 1, -1};
            zMatrix.setValues(driveZ);
            zMatrix.scalar(zInches);
        }

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            frontLeftTarget = robot.getCurrentPosition(Robot.Motor.FRONT_LEFT) + (int)(fin.getValue(0, 0) * COUNTS_PER_INCH);
            frontRightTarget = robot.getCurrentPosition(Robot.Motor.FRONT_RIGHT) + (int)(fin.getValue(1, 0) * COUNTS_PER_INCH);
            backLeftTarget = robot.getCurrentPosition(Robot.Motor.FRONT_RIGHT) + (int)(fin.getValue(0, 1) * COUNTS_PER_INCH);
            backRightTarget = robot.getCurrentPosition(Robot.Motor.FRONT_RIGHT) + (int)(fin.getValue(1, 1) * COUNTS_PER_INCH);
            robot.setTargetPosition(Robot.Motor.FRONT_LEFT, frontLeftTarget);
            robot.setTargetPosition(Robot.Motor.FRONT_RIGHT, frontRightTarget);
            robot.setTargetPosition(Robot.Motor.BACK_LEFT, backLeftTarget);
            robot.setTargetPosition(Robot.Motor.BACK_RIGHT, backRightTarget);

            // Turn On RUN_TO_POSITION
            robot.setRunMode(Robot.Motor.FRONT_LEFT, DcMotor.RunMode.RUN_TO_POSITION);
            robot.setRunMode(Robot.Motor.FRONT_RIGHT, DcMotor.RunMode.RUN_TO_POSITION);
            robot.setRunMode(Robot.Motor.BACK_LEFT, DcMotor.RunMode.RUN_TO_POSITION);
            robot.setRunMode(Robot.Motor.BACK_RIGHT, DcMotor.RunMode.RUN_TO_POSITION);
            // reset the timeout time and start motion.
            runtime.reset();
//            if (!rotate) {
//                drive.setPower(x, y);
//            } else {
//                drive.setRotationPower(x, y);
//            }

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.isBusy(Robot.Motor.FRONT_LEFT) && robot.isBusy(Robot.Motor.FRONT_RIGHT) &&
                            robot.isBusy(Robot.Motor.BACK_LEFT) && robot.isBusy(Robot.Motor.BACK_RIGHT))) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d : %7d : %7d : %7d",
                        frontLeftTarget,
                        frontRightTarget,
                        backLeftTarget,
                        backRightTarget);
                telemetry.addData("Path2",  "Starting at %7d : %7d : %7d : %7d",
                        robot.getCurrentPosition(Robot.Motor.FRONT_LEFT),
                        robot.getCurrentPosition(Robot.Motor.FRONT_RIGHT),
                        robot.getCurrentPosition(Robot.Motor.BACK_LEFT),
                        robot.getCurrentPosition(Robot.Motor.BACK_RIGHT));
                telemetry.update();
            }

            // Stop all motion;
            robot.setPower(Robot.Motor.FRONT_LEFT, 0);
            robot.setPower(Robot.Motor.FRONT_RIGHT, 0);
            robot.setPower(Robot.Motor.BACK_LEFT, 0);
            robot.setPower(Robot.Motor.BACK_RIGHT, 0);

            // Turn off RUN_TO_POSITION
            robot.setRunMode(Robot.Motor.FRONT_LEFT, DcMotor.RunMode.RUN_USING_ENCODER);
            robot.setRunMode(Robot.Motor.FRONT_RIGHT, DcMotor.RunMode.RUN_USING_ENCODER);
            robot.setRunMode(Robot.Motor.BACK_LEFT, DcMotor.RunMode.RUN_USING_ENCODER);
            robot.setRunMode(Robot.Motor.BACK_RIGHT, DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    public void runMotors(int frontLeftTarget, int frontRightTarget, int backLeftTarget, int backRightTarget) {

    }
}
