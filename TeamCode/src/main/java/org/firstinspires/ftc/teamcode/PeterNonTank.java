package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.BreakoutMotor.Direction.MOTOR_F;
import static org.firstinspires.ftc.teamcode.BreakoutMotor.Direction.MOTOR_R;

/**
 * This class is used for the main game to drive the robot using the controllers.
 **/
@TeleOp(name = "DriveNonTank", group = "Pushbot")

public class PeterNonTank extends OpMode {

    //Servo Breakout code definition
   /* private BreakoutServo servoA = new BreakoutServo();
    private BreakoutServo servoB = new BreakoutServo();
    private BreakoutServo servoC = new BreakoutServo();
    private BreakoutServo servoD = new BreakoutServo(); */

    private BreakoutMotor motorLeft = new BreakoutMotor();
    private BreakoutMotor motorRight = new BreakoutMotor();
    private BreakoutMotor motorHorizontal1 = new BreakoutMotor();
    private BreakoutMotor motorSweeperArm = new BreakoutMotor();
    private BreakoutMotor motorSweeper = new BreakoutMotor();
    private BreakoutMotor motorHorizontal2 = new BreakoutMotor();

    //Gyro Breakout code definition
    //private BreakoutREVGyro gyroA = new BreakoutREVGyro();

    //Timer definition
    //private ElapsedTime timer = new ElapsedTime();

    @Override
    public void init() {

        //Broken out servo class
       /* servoA.set(hardwareMap.servo.get("servoA"));
        servoB.set(hardwareMap.servo.get("servoB"));
        servoA.setDirection(SERVO_F);
        servoB.setDirection(SERVO_R);
        servoA.setPosition(0);
        servoB.setPosition(0);
        servoC.set(hardwareMap.servo.get("servoC"));
        servoD.set(hardwareMap.servo.get("servoD"));
        servoC.setDirection(SERVO_F);
        servoD.setDirection(SERVO_R);
        servoC.setPosition(0);
        servoD.setPosition(0);  */

        //Broken out motor class
        motorLeft.set(hardwareMap.dcMotor.get("motorLeft"));
        motorRight.set(hardwareMap.dcMotor.get("motorRight"));
        motorSweeper.set(hardwareMap.dcMotor.get("motorSweeper"));
        motorHorizontal1.set(hardwareMap.dcMotor.get("motorHorizontal"));
        motorHorizontal2.set(hardwareMap.dcMotor.get("motorHorizontalBack"));
        motorSweeperArm.set(hardwareMap.dcMotor.get("motorSweeperArm"));
        motorLeft.setDirection(MOTOR_R);
        motorRight.setDirection(MOTOR_R);
        motorHorizontal1.setDirection(MOTOR_F);
        motorHorizontal2.setDirection(MOTOR_R);
        motorLeft.setPower(0);
        motorRight.setPower(0);

        //Broken out Gyro class
        //gyroA.set(hardwareMap.get(gyroA.IMU, "gyroA"));
        //telemetry.addLine("Calibrating: DO NOT MOVE!");
        // gyroA.calibrate();
        telemetry.clearAll();

    }

    @Override
    public void start() {

        //Motor start
        motorLeft.setPower(0);
        motorRight.setPower(0);

        //Servo start
        // servoA.setPosition(0);
        // servoB.setPosition(0);

    }

    @Override
    public void loop() {

        //Gamepad 1
        float leftStick1x = gamepad1.left_stick_x;
        float leftStick1y = gamepad1.left_stick_y;
        float rightStick1x = gamepad1.right_stick_x;
        float rightStick1y = gamepad1.right_stick_y;
        //Gamepad 2
        float leftStick2x = gamepad2.left_stick_x;
        float leftStick2y = gamepad2.left_stick_y;
        float rightStick2x = gamepad2.right_stick_x;
        float rightStick2y = gamepad2.right_stick_y;
        boolean leftBumper = gamepad2.left_bumper;
        boolean rightBumper = gamepad2.right_bumper;

        //Move the motors//
//        float leftRightDirection = (float) (rightStick1x * 0.5);
//        if (leftRightDirection <= 0) {
//            motorLeft.setPower((0.5 * -leftStick1y) + leftRightDirection);
//            motorRight.setPower((0.5 * -leftStick1y) - leftRightDirection);
//        } else {
//            motorLeft.setPower((0.5 * -leftStick1y) - leftRightDirection);
//            motorRight.setPower((0.5 * -leftStick1y) + leftRightDirection);
//        }
        // Peter, I think the code would be better something like this...we can work on the names,
        // but it at least explains what is going on.

        // I do see where if you use the power all the way forward or all the way backward, and use
        // the left/right all the way one way or the other, the speed of one motor is 0 and the other
        // motor is -1/+1, so it basically spins in a circle...not sure that's what you intended.

        // I think we can get this figured out, we should probably discuss what it was you were
        // trying to do with it
        float leftRightDirection = (float) (rightStick1x * 0.5);
        double reversePowerStickAndMakeItHalf = 0.5 * -leftStick1y;
        if (leftRightDirection <= 0) {
            motorLeft.setPower(reversePowerStickAndMakeItHalf + leftRightDirection);
            motorRight.setPower(reversePowerStickAndMakeItHalf - leftRightDirection);
        } else {
            motorLeft.setPower(reversePowerStickAndMakeItHalf - leftRightDirection);
            motorRight.setPower(reversePowerStickAndMakeItHalf + leftRightDirection);
        }
        if (gamepad2.y) {
            motorSweeperArm.setPower(-1);
        } else if (gamepad2.a) {
            motorSweeperArm.setPower(1);
        } else {
            motorSweeperArm.setPower(0);
        }
        if (gamepad2.x) {
            motorHorizontal1.setPower(1);
            motorHorizontal2.setPower(1);
        } else if (gamepad2.b) {
            motorHorizontal1.setPower(-1);
            motorHorizontal2.setPower(-1);
        } else {
            motorHorizontal1.setPower(0);
            motorHorizontal2.setPower(0);
        }
        if (leftBumper) {
            motorSweeper.setPower(-0.5);
        } else if (rightBumper) {
            motorSweeper.setPower(0.5);
        } else {
            motorSweeper.setPower(0);
        }

        //Set the servos
        // servoA.setPosition(leftStick2y);
        // servoB.setPosition(rightStick1y);

        //Telemetry
        telemetry.addData("Left Stick X 1", leftStick1x);
        telemetry.addData("Left Stick Y 1", leftStick1y);
        telemetry.addData("Right Stick X 1", rightStick1x);
        telemetry.addData("Right Stick Y 1", rightStick1y);
        telemetry.addData("Left Stick X 2", leftStick2x);
        telemetry.addData("Left Stick Y 2", leftStick2y);
        telemetry.addData("Right Stick X 2", rightStick2x);
        telemetry.addData("Right Stick Y 2", rightStick2y);
        // telemetry.addData("Gyro Heading", gyroA.getPos());
        //telemetry.addData("Gyro Orientation", gyroA.getOrient());
        // telemetry.addData("Gyro Velocity", gyroA.getVel());
        // telemetry.addData("Gyro Acceleration", gyroA.getAccel());

    }

    @Override
    public void stop() {

        //Motor stop
        motorLeft.setPower(0);
        motorRight.setPower(0);
        motorSweeper.setPower(0);
        motorHorizontal1.setPower(0);
        motorSweeperArm.setPower(0);
        motorHorizontal2.setPower(0);

        //Servo stop
        // servoA.setPosition(0);
        // servoB.setPosition(0);
        // servoC.setPosition(0);
        // servoD.setPosition(0);

    }

}