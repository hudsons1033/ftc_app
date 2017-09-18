package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOp: Teleop Tank", group="Pushbot")

public class TeleOp extends OpMode {

  private DcMotor motorLeft;
  private DcMotor motorRight;
  private DcMotor motorCatapult;
  private DcMotor motorSweeper;

  private Wheels wheels;

  public TeleOp() {
    wheels = new Wheels();
  }

  @Override
  public void init() {
    motorLeft  = hardwareMap.dcMotor.get("MotorLeft");
    motorRight = hardwareMap.dcMotor.get("MotorRight");
    wheels.init(motorLeft, motorRight);

    motorCatapult = hardwareMap.dcMotor.get("MotorCatapult");
    motorCatapult.setDirection(DcMotor.Direction.FORWARD);
    motorCatapult.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);

    motorSweeper  = hardwareMap.dcMotor.get("MotorSweeper");
    motorSweeper.setDirection(DcMotor.Direction.REVERSE);
    motorSweeper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
  }

  @Override
  public void start() {
    wheels.start();
    motorCatapult.setPower(0.0);
    motorSweeper.setPower(0.0);
  }

  @Override
  public void loop() {
    // Gamepad sticks are inverted.
    wheels.move(-gamepad1.left_stick_y, -gamepad1.right_stick_y);

    motorCatapult.setPower(gamepad2.left_trigger);
    motorSweeper.setPower(gamepad2.right_trigger);

    telemetry.addData("Text", "*** Robot Data***");
  }

  @Override
  public void stop() {
    wheels.stop();
    motorCatapult.setPower(0.0);
    motorSweeper.setPower(0.0);
  }

}
