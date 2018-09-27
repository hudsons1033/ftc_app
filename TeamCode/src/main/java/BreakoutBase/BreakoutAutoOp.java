package BreakoutBase;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import static BreakoutBase.BreakoutMotor.Direction.MOTOR_F;
import static BreakoutBase.BreakoutMotor.Direction.MOTOR_R;
import static BreakoutBase.BreakoutServo.Direction.SERVO_FORWARD;
import static BreakoutBase.BreakoutServo.Direction.SERVO_REVERSE;

/**
 * This class is used for the Autonomous segment of the game. No controllers are to be used.
 **/
@Autonomous(name = "BreakoutAutonomous", group = "pushbot")

public class BreakoutAutoOp extends OpMode {

    //Servo Breakout code definition
    private BreakoutServo servoA = new BreakoutServo();
    private BreakoutServo servoB = new BreakoutServo();

    //Motor Breakout code definition
    private BreakoutMotor motorA = new BreakoutMotor();
    private BreakoutMotor motorB = new BreakoutMotor();

    //Gyro Breakout code definition
    private BreakoutGyro gyroA = new BreakoutGyro();

    //Timer definition
    private ElapsedTime timer = new ElapsedTime();

    public void init() {

        //Broken out servo class
        servoA.set(hardwareMap.servo.get("servoA"));
        servoB.set(hardwareMap.servo.get("servoB"));
        servoA.setDirection(SERVO_REVERSE);
        servoB.setDirection(SERVO_FORWARD);
        servoA.setPosition(0);
        servoB.setPosition(0);

        //Broken out motor class
        motorA.set(hardwareMap.dcMotor.get("motorA"));
        motorB.set(hardwareMap.dcMotor.get("motorB"));
        motorA.setDirection(MOTOR_F);
        motorB.setDirection(MOTOR_R);
        motorA.setPower(0);
        motorB.setPower(0);

        //Broken out Gyro class
        gyroA.set(hardwareMap.gyroSensor.get("gyroA"));
        telemetry.addLine("Calibrating: DO NOT MOVE!");
        gyroA.calibrate();
        timer.reset();
        while (gyroA.gyroSensor.isCalibrating()) {
            telemetry.addData("Calibrating: ", Math.round(timer.seconds()) + " seconds");
            telemetry.update();
            try {
                wait(50);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

        }

    }

    @Override
    public void loop() {

        //Run autonomous code
        servoA.setPosition(1);
        servoB.setPosition(1);
        motorA.setPower(1);
        motorB.setPower(1);
        try {
            wait(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        servoA.setPosition(0);
        servoB.setPosition(0);
        motorA.setPower(0);
        motorB.setPower(0);
        try {
            wait(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

    }

}
