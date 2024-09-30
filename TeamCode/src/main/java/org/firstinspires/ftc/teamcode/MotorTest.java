package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import Wheelie.Pose2D;

@TeleOp
public class MotorTest extends LinearOpMode {
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

            if(gamepad1.a){
                drive.moveWithPower(1,0,0,0);
            } else
            if(gamepad1.b){
                drive.moveWithPower(0,1,0,0);
            }else
            if(gamepad1.y){
                drive.moveWithPower(0,0,1,0);
            }else
            if(gamepad1.x){
                drive.moveWithPower(0,0,0,1);
            } else{
                drive.moveWithPower(0);
            }
        }
    }
}