package org.firstinspires.ftc.teamcode.mecanum;

import org.firstinspires.ftc.teamcode.breakout.Robot;

public class Mecanum {

    private Robot robot;
    private float finFL;
    private float finFR;
    private float finBL;
    private float finBR;

    public Mecanum(Robot robot) {
        this.robot = robot;
    }

    public float[] setPower(float x, float y, float z) {
        float finFL = y + z + x;
        float finFR = y - z - x;
        float finBL = y + z - x;
        float finBR = y - z + x;
        normalize(finFL, finFR, finBL, finBR);
        robot.setPower(Robot.Motor.FRONT_LEFT, this.finFL);
        robot.setPower(Robot.Motor.FRONT_RIGHT, this.finFR);
        robot.setPower(Robot.Motor.BACK_LEFT, this.finBL);
        robot.setPower(Robot.Motor.BACK_RIGHT, this.finBR);
        return new float[]{this.finFL, this.finFR, this.finBL, this.finBR};
    }

    private void normalize(float fl, float fr, float bl, float br) {
        float t0 = Math.max(Math.abs(fl), Math.abs(fr));
        float t1 = Math.max(Math.abs(bl), Math.abs(br));
        float max = Math.max(t0, t1);

        if (max < 1) {
            finFL = fl;
            finFR = fr;
            finBL = bl;
            finBR = br;
        } else {
            finFL = fl/max;
            finFR = fr/max;
            finBL = bl/max;
            finBR = br/max;
        }
    }

    @Deprecated
    public void setRotationPower(float left, float right) {
        float leftside = right-left;
        float rightside = left-right;

        robot.setPower(Robot.Motor.FRONT_LEFT, leftside);
        robot.setPower(Robot.Motor.FRONT_RIGHT, rightside);
        robot.setPower(Robot.Motor.BACK_LEFT, leftside);
        robot.setPower(Robot.Motor.BACK_RIGHT, rightside);
    }
}
