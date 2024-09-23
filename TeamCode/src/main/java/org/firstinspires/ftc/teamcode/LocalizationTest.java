package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import Wheelie.Pose2D;

@TeleOp
public class LocalizationTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecDrivebase drive = new MecDrivebase(hardwareMap, new Pose2D(0, 0, 0));
        telemetry.addLine("Initialized");
        telemetry.addLine(drive.getPoseString());
        drive.update();
        telemetry.addLine(drive.getPoseString());
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            drive.update();
            telemetry.addLine(drive.getPoseString());
            telemetry.addData("Horizontal wheel", drive.getHoriOdom());
            telemetry.addData("Vertical wheel", drive.getVertOdom());
            telemetry.update();
        }
    }
}