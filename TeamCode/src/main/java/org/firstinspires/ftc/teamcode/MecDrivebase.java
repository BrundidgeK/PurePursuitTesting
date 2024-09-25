package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import Wheelie.PID;
import Wheelie.Pose2D;

public class MecDrivebase {
    private DcMotorEx[] motors = new DcMotorEx[4];
    private static final String[] MOTOR_NAMES = {
            "FL",
            "BL",
            "BR",
            "FR"
    };
    private DcMotorSimple.Direction[] directions = {
            DcMotorSimple.Direction.REVERSE,
            DcMotorSimple.Direction.REVERSE,
            DcMotorSimple.Direction.FORWARD,
            DcMotorSimple.Direction.FORWARD
    };

    private Localization localization;
    private PID mxPID = new PID(1.0/36.0, 0, 0), myPID = new PID(1.0/36.0, 0, 0),
                hPID = new PID(1./90., 0, 0); //TODO re-evaluate these values

    //The max speed of the motors
    public static final double SPEED_PERCENT = 1;

    public MecDrivebase(HardwareMap hw, Pose2D startPose)
    {
        myPID.setStartTime();
        mxPID.setStartTime();
        hPID.setStartTime();
        myPID.capI(5); //TODO Placeholder
        mxPID.capI(5); //TODO Placeholder
        hPID.capI(5); //TODO Placeholder

        localization = new Localization(hw, startPose);

        for (int i = 0; i < motors.length; i++) {
            motors[i] = hw.get(DcMotorEx.class, MOTOR_NAMES[i]);
            motors[i].setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            motors[i].setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            motors[i].setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            motors[i].setDirection(directions[i]);
        }
    }

    /** Sets all motors to the same power */
    public void moveWithPower(double power) {
        for (DcMotorEx m : motors) {
            m.setPower(power * SPEED_PERCENT);
        }
    }

    /** Sets all motors to their respective powers */
    public void moveWithPower(double fl, double bl, double br, double fr) {
        motors[0].setPower(Range.clip(fl, -1, 1) * SPEED_PERCENT);
        motors[1].setPower(Range.clip(bl, -1, 1) * SPEED_PERCENT);
        motors[2].setPower(Range.clip(fr, -1, 1) * SPEED_PERCENT);
        motors[3].setPower(Range.clip(br, -1, 1) * SPEED_PERCENT);
    }

    /** Sets motor powers so drivebase can move towards target based on input (usually from the PathFollower class)*/
    public void  moveTo(double forward, double strafe, double heading){
        double movementAngle = Math.atan2(strafe, forward) - localization.getAngle();
        double x = forward * Math.cos(movementAngle) - strafe * Math.sin(movementAngle);
        double y = forward * Math.sin(movementAngle) + strafe * Math.cos(movementAngle);
        double h =0;// Range.clip(hPID.pidCalc(heading, .1), -.5, .5);


        double length = x + y;

        if(length > 1){
            x /= length;
            y /= length;
        }

        if (x > .1 && y >.1 && (h > x || h > y))
            h = 0;

        moveWithPower(
                x + y + h,
                x - y + h,
                x + y - h,
                x - y - h
        );
    }
    /** Sets motor powers so drivebase can move towards target based on input (usually from the PathFollower class)*/
    public void moveTo(Pose2D move){
        moveTo(move.x, move.y, move.h);
    }

    /** Sets motor powers so drivebase can move towards target using PID (for when the lookahead is shrinking)
     * @param move output from PathFollwer class, followPath method
     * @param target the robot's target location
     * @param startTime the start time when this method was first called for the specified target
     */
    public void moveToPID(Pose2D move, Pose2D target, double startTime){
        double forward = move.x,
                strafe = move.y,
                heading = move.h;
        Pose2D tar = new Pose2D(getPose().x + forward, getPose().y + strafe, getPose().h + heading);

        //double movementAngle = Math.atan2(strafe, forward) - localization.getAngle();
        //double x = Math.cos(movementAngle) * forward;
        //double y = Math.sin(movementAngle) * strafe;

        double x = mxPID.pidCalc(tar.x, getPose().x),
        y = myPID.pidCalc(tar.y, getPose().y);
        heading = hPID.pidCalc(tar.h, getPose().h);


        double length = x + y;

        if(length > 1) {
            x /= length;
            y /= length;
            heading /= length;
        }

        moveWithPower(
                x + y + heading,
                x - y + heading,
                x + y - heading,
                x - y - heading
        );
    }

    public void resetPID(){
        mxPID.resetI();
        myPID.resetI();
        hPID.resetI();
    }

    public double getPower(int index) {
        return motors[index].getPower();
    }

    public String getPowers() {
        return motors[0].getPower() + ", " +
                motors[1].getPower() + ", " +
                motors[2].getPower() + ", " +
                motors[3].getPower();
    }

    public Pose2D getPose(){
        return localization.currentPosition;
    }
    public String getPoseString(){
        return localization.currentPosition.x + ", " +
                localization.currentPosition.y + ", " +
                localization.currentPosition.h;
    }

    public int getHoriOdom(){
        return localization.getHori();
    }
    public int getVertOdom(){
        return localization.getVert();
    }

    public void update(){
        localization.update();
    }
}
