package BreakoutBase;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class BreakoutAutoOp extends OpMode {

    //Direction Constants
    private DcMotor.Direction MOTOR_FORWARD = DcMotor.Direction.FORWARD;
    private Servo.Direction SERVO_FORWARD = Servo.Direction.FORWARD;
    private DcMotor.Direction MOTOR_REVERSE = DcMotor.Direction.REVERSE;
    private Servo.Direction SERVO_REVERSE = Servo.Direction.REVERSE;

    //Servo Breakout code definition
    private ServoBreakout servoA;
    private ServoBreakout servoB;

    //Motor Breakout code definition
    private MotorBreakout motorA;
    private MotorBreakout motorB;

    public void init() {

        //Broken out servo class
        Servo servo = hardwareMap.servo.get("servo");
        Servo servo2 = hardwareMap.servo.get("servo2");
        servoA = new ServoBreakout(servo2);
        servoB = new ServoBreakout(servo);
        servoA.setDirection(SERVO_REVERSE);
        servoB.setDirection(SERVO_FORWARD);
        servoA.setPosition(0);
        servoB.setPosition(0);

        //Broken out motor class
        DcMotor motor = hardwareMap.dcMotor.get("motor");
        DcMotor motor2 = hardwareMap.dcMotor.get("motor2");
        motorA = new MotorBreakout(motor2);
        motorB = new MotorBreakout(motor);
        motorA.setDirection(MOTOR_REVERSE);
        motorB.setDirection(MOTOR_FORWARD);
        motorA.setPower(0);
        motorB.setPower(0);

    }

    @Override
    public void loop() {

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
