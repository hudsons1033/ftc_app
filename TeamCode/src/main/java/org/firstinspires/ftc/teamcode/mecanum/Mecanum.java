package org.firstinspires.ftc.teamcode.mecanum;

import org.firstinspires.ftc.teamcode.breakout.Robot;

public class Mecanum {

    private Robot robot;

    public Mecanum(Robot robot) {
        this.robot = robot;
    }

    public void setPower(float x, float y) {
        robot.setPower(Robot.Motor.FRONT_LEFT, y+x);
        robot.setPower(Robot.Motor.FRONT_RIGHT, y-x);
        robot.setPower(Robot.Motor.BACK_LEFT, y-x);
        robot.setPower(Robot.Motor.BACK_RIGHT, y+x);
    }

    public void setRotationPower(float left, float right) {
        float leftside = right-left;
        float rightside = left-right;

        robot.setPower(Robot.Motor.FRONT_LEFT, leftside);
        robot.setPower(Robot.Motor.FRONT_RIGHT, rightside);
        robot.setPower(Robot.Motor.BACK_LEFT, leftside);
        robot.setPower(Robot.Motor.BACK_RIGHT, rightside);
    }
}
