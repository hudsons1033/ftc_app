package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;




public class NewAuto extends OpMode {

    private static double Speed = 0.4;
    private static double InnerSpeed = Speed * ConfigValues.InsideRatio;

    private static double SecondsPerFoot = 0.5 / Speed;

private enum State {
    Begin,
    Depart,
    Turn1,
    Straight,
    Turn2,
    Approach,
    Dock
}

    private static double DepartDist   =  0.0;
    private static double Turn1Dist    =  ConfigValues.FeetPerCircle / 4; // 45
    // degrees
    private static double StraightDist =  4.7;
    private static double Turn2Dist    =  ConfigValues.FeetPerCircle / 4; // 45
    //degrees
    private static double ApproachDist =  1.4;

    private static double DepartDur   =  DepartDist   * SecondsPerFoot;
    private static double Turn1Dur    =  Turn1Dist    * SecondsPerFoot;
    private static double StraightDur =  StraightDist * SecondsPerFoot;
   private static double Turn2Dur    =  Turn2Dist    * SecondsPerFoot;
    private static double ApproachDur =  ApproachDist * SecondsPerFoot;

    private static double DepartEnd   = DepartDur;
    private static double Turn1End    = DepartEnd   + Turn1Dur;
    private static double StraightEnd = Turn1End    + StraightDur;
    private static double Turn2End    = StraightEnd + Turn2Dur;
    private static double ApproachEnd = Turn2End    + ApproachDur;

    private ElapsedTime timer = new ElapsedTime();

    private DcMotor motorLeft;
    private DcMotor motorRight;
    //private DcMotor Flywheel;



   // private Servo PressThing;


    private NewAuto.State state = NewAuto.State.Begin;

    private double leftPower  = 0.0;
    private double rightPower = 0.0;

    private double leftPosFt  = 0.0;
    private double rightPosFt = 0.0;

    private void setLeftPower(double pwr) {
        leftPower = Range.clip(pwr,  -1, 1);
        motorLeft.setPower(leftPower);

    }

    private void setRightPower(double pwr) {
        rightPower = Range.clip(pwr,  -1, 1);
        motorRight.setPower(rightPower);

    }

    private void setPower(double left, double right) {
        setLeftPower(left);
        setRightPower(right);
    }

    private void setWheelMode(DcMotor.RunMode mode) {
        motorRight.setMode(mode);
        motorLeft.setMode(mode);

    }

    private static int Clicks(double ft)
    { return (int)(ft / ConfigValues.FeetPerWheelRev * ConfigValues.ClicksPerRev
            + 0.5); }

    private void setTarget(double leftFt, double rightFt) {
        int left  = Clicks(leftFt);
        int right = Clicks(rightFt);
        motorLeft.setTargetPosition(left);
        motorRight.setTargetPosition(right);

    }

    private void move(double ft) {
        leftPosFt  += ft;
        rightPosFt += ft;
        setPower(Speed, Speed);
        setTarget(leftPosFt, rightPosFt);
   }

    private void turnLeft(double ft) {
        leftPosFt  += ft * ConfigValues.InsideRatio;
        rightPosFt += ft;
        setPower(InnerSpeed, Speed);
        setTarget(leftPosFt, rightPosFt);
    }

    private void turnRight(double ft) {
        leftPosFt  += ft;
        rightPosFt += ft * ConfigValues.InsideRatio;
        setPower(Speed, InnerSpeed);
        setTarget(leftPosFt, rightPosFt);
    }

    public NewAuto() { }

    @Override
    public void init() {
        motorLeft  = hardwareMap.dcMotor.get("MotorLeft");
        motorRight = hardwareMap.dcMotor.get("MotorRight");
       //Flywheel = hardwareMap.dcMotor.get("Flywheel");
       //PressThing = hardwareMap.servo.get("presser");


        motorLeft.setDirection(DcMotor.Direction.FORWARD);
        motorRight.setDirection(DcMotor.Direction.REVERSE);
       // Flywheel.setDirection(DcMotor.Direction.FORWARD);
       // PressThing.setDirection(Servo.Direction.FORWARD);





    }

    @Override
    public void start() {
        setPower(0.0, 0.0);
        leftPosFt  = 0.0;
        rightPosFt = 0.0;
        state = NewAuto.State.Begin;
    }

    @Override
    public void loop() {
        switch (state) {
            //case Begin:
               // timer.reset();
               // move(DepartDist);
               // state = NewAuto.State.Depart;
                // fall thru
            //case Depart:
                //if (timer.time() < DepartEnd)
                   // break;
               // turnLeft(Turn1Dist);
               // state = NewAuto.State.Turn1;
                // fall thru
            case Turn1:
                if (timer.time() < Turn1End)
                    break;
                move(StraightDist);
                state = NewAuto.State.Straight;
                // fall thru
            case Straight:
                if (timer.time() < StraightEnd)
                    break;
                turnLeft(Turn2Dist);
                state = NewAuto.State.Turn2;
                // fall thru
            case Turn2:
                if (timer.time() < Turn2End)
                    break;
                move(ApproachDist);
                state = NewAuto.State.Approach;
                // fall thru
            case Approach:
                if (timer.time() < ApproachEnd)
                    break;
                setPower(0.0, 0.0);
                state = NewAuto.State.Dock;
                // fall thru
            case Dock:
                break;
        }

        telemetry.addData("state", String.format("state: %s", state));
        telemetry.addData("left",  String.format("%.2f", leftPosFt));
        telemetry.addData("right", String.format("%.2f", rightPosFt));
    }

    @Override
    public void stop() { setPower(0.0, 0.0); }

} // AutoLongLeft
