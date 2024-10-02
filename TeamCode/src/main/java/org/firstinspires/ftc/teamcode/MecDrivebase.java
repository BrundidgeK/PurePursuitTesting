package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import java.util.Arrays;

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
    /*private PID mxPID = new PID(1.0/36.0, 0, 0), myPID = new PID(1.0/36.0, 0, 0),
                hPID = new PID(1./90., 0, 0); //TODO re-evaluate these values

     */

    private PIDController xPID;
    private PIDController yPID;
    private PIDController hPID;

    private PathFollow follower;

    //The max speed of the motors
    public static double SPEED_PERCENT = 1;

    public MecDrivebase(HardwareMap hw, Pose2D startPose)
    {
        xPID = new PIDController(1.0/24.0,0.01,.125);
        yPID = new PIDController(1.0/24.0,0.01,.125);
        hPID = new PIDController(1.0/Math.PI,0,0);
       /* myPID.setStartTime();
        mxPID.setStartTime();
        hPID.setStartTime();
        myPID.capI(5); //TODO Placeholder
        mxPID.capI(5); //TODO Placeholder
        hPID.capI(5); //TODO Placeholder

        */

        localization = new Localization(hw, startPose);

        for (int i = 0; i < motors.length; i++) {
            motors[i] = hw.get(DcMotorEx.class, MOTOR_NAMES[i]);
            motors[i].setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            motors[i].setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            motors[i].setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            motors[i].setDirection(directions[i]);
        }
    }

    public MecDrivebase(HardwareMap hw, Pose2D startPose, double maxSpeed)
    {
        xPID = new PIDController(1.0/24.0,0.01,.125);
        yPID = new PIDController(1.0/24.0,0.01,.125);
        hPID = new PIDController(1.0/Math.PI,0,0);
        /*myPID.setStartTime();
        mxPID.setStartTime();
        hPID.setStartTime();
        myPID.capI(5); //TODO Placeholder
        mxPID.capI(5); //TODO Placeholder
        hPID.capI(5); //TODO Placeholder

         */

        localization = new Localization(hw, startPose);

        for (int i = 0; i < motors.length; i++) {
            motors[i] = hw.get(DcMotorEx.class, MOTOR_NAMES[i]);
            motors[i].setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            motors[i].setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            motors[i].setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            motors[i].setDirection(directions[i]);
        }

        SPEED_PERCENT = maxSpeed;
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
    public void moveTo(double forward, double strafe, double heading){
        double movementAngle = Math.atan2(strafe, forward) - localization.getAngle();
        double x = forward * Math.cos(movementAngle) - strafe * Math.sin(movementAngle);
        double y = forward * Math.sin(movementAngle) + strafe * Math.cos(movementAngle);
        double h = 0;//Math.abs(heading) <= Math.toRadians(5) ? 0 : heading * .25;

        double length = Math.abs(x) + Math.abs(y);

        /*if (heading != 0 && length > .75){
            x /= length;
            y /= length;
            x *= .75;
            y *= .75;
        } else if(length > 1){
            x /= length;
            y /= length;
        }

         */

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
     */
    public void moveToPID(Pose2D move){
        double forward = move.x,
                strafe = move.y,
                heading = move.h;

        double x = Math.cos(localization.getAngle()) * forward + Math.sin(localization.getAngle()) * strafe;
        double y = Math.cos(localization.getAngle()) * strafe - Math.sin(localization.getAngle()) * forward;

        double movementAngle = Math.atan2(strafe, forward) - localization.getAngle(),
                distance = Math.hypot(x, y);

        x = xPID.calculateResponse(distance * Math.cos(movementAngle));
        y = yPID.calculateResponse(distance * Math.sin(movementAngle));

        double length = Math.hypot(x,y);

        if(length > 1) {
            x /= length;
            y /= length;
            if(Math.abs(heading) > Math.toRadians(5)){
                x*= .75;
                y *= .75;
            }
        }

        if(length > 5){
            if(heading > Math.toRadians(5))
                heading = Range.clip(hPID.calculateResponse(-heading), -.25, .25) * ((x+y)/(2.0 * length));
        }
        else
            heading = Range.clip(hPID.calculateResponse(-heading), -.5, .5);



        moveWithPower(
                x + y + heading,
                x - y + heading,
                x + y - heading,
                x - y - heading
        );
    }


    public void rotate(double heading){
        //TODO create a rotation on spot method
    }

    public void resetPID(){
        xPID.reset();
        yPID.reset();
        hPID.reset();
    }

    public void setFollower(PathFollow f){
        follower = f;
        resetPID();
    }
    public PathFollow getFollower(){
        return follower;
    }

    public boolean targetReached(Pose2D target){
        return Math.hypot(target.x-getPose().x, target.y-getPose().y) <= 2 &&
                Math.abs(target.h-getPose().h) <= Math.toRadians(5);
    }
    public void concludePath(){
        moveWithPower(0);
        follower = null;
    }

    public double getPower(int index) {
        return motors[index].getPower();
    }

    public String getPowers() {
        return String.format("%.2f", motors[0].getPower()) + ", " +
                String.format("%.2f", motors[1].getPower()) + ", " +
                String.format("%.2f", motors[2].getPower()) + ", " +
                String.format("%.2f", motors[3].getPower()) + ", ";
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

    Pose2D m;
    public Pose2D followerValues(){
        return m;
    }

    public void update(){
        localization.update();

        if(follower != null) {
            m = follower.followPath(getPose());
            moveTo(m.x, m.y, m.h);

            if(targetReached(follower.getLastPoint()))
                concludePath();
        }
    }
    public void update(int i){
        localization.update();

        if(follower != null) {
            m = follower.followPath(getPose());
            moveToPID(m);

            if(targetReached(follower.getLastPoint()))
                concludePath();
        }
    }

    @Override
    public String toString() {
        return "MecDrivebase{" +
                "motors=" + Arrays.toString(motors) +
                ", directions=" + Arrays.toString(directions) +
                ", localization=" + localization +
                ", mxPID=" + xPID +
                ", myPID=" + yPID +
                ", hPID=" + hPID +
                ", follower=" + follower +
                ", m=" + m +
                '}';
    }
}
