package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import Wheelie.PID;
import Wheelie.Pose2D;

@TeleOp
public class PIDTest extends LinearOpMode {
    PID pid = new PID(1, 0,0);
    double i = 0,
    maxI = 5,
    prevTime = 0,
    prevErr = 0;

    ElapsedTime time;
    @Override
    public void runOpMode() throws InterruptedException {
        MecDrivebase drive = new MecDrivebase(hardwareMap, new Pose2D(0, 0, 0));
        telemetry.addLine("Initialized");
        telemetry.addLine(drive.getPoseString());
        drive.update();
        telemetry.addLine(drive.getPoseString());
        telemetry.update();

        waitForStart();

        pid.capI(5);

        while (opModeIsActive()) {
            drive.update();
            telemetry.addData("PID", pid.pidCalc(24, 1));
            telemetry.addData("PID", pidCalc(24, 1));
            telemetry.addLine(drive.getPoseString());
            telemetry.addData("Motor", drive.getPowers());
            telemetry.update();
        }
    }
    public double pidCalc (double target, double currPos) {
        double currErr = target - currPos;
        double p = pid.kP * currErr;
        telemetry.addLine(""+p);

        i += pid.kI * (currErr * ((time.time()) - prevTime));
        telemetry.addLine(""+i);

        if (!(maxI == maxI)) {
            i = (i > maxI) ? maxI : i;
            i = (i < -maxI) ? -maxI : i;
        }

        telemetry.addLine(""+i);
        double d = pid.kD * (currErr - prevErr) / ((time.time()) - prevTime);

        telemetry.addLine(""+(System.currentTimeMillis()/1000 - prevTime));
        prevErr = currErr;
        prevTime = (time.time());

        return p + i + d;
    }
}