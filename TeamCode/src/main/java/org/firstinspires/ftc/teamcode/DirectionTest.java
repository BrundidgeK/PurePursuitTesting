package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import Wheelie.Path;
import Wheelie.Pose2D;

@Autonomous
public class DirectionTest extends LinearOpMode {
    private MecDrivebase drive;
    private PathFollow follower;

    private Pose2D[] points = new Pose2D[] {
            new Pose2D(0, 0,0),
            new Pose2D(24, 0, 0),
            new Pose2D(24, 24, 0)
    };

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2D start = new Pose2D(0,0,0);

        drive = new MecDrivebase(hardwareMap, start, .5);
        String dir = "forward";

        while (opModeInInit() && !isStopRequested()){
            if(gamepad1.dpad_up){
                points[1] = new Pose2D(12, 0, 0);
                points[2] = new Pose2D(24, 0, 0);
                dir = "forward";
            } else if(gamepad1.dpad_down){
                points[1] = new Pose2D(-12, 0, 0);
                points[2] = new Pose2D(-24, 0, 0);
                dir = "backward";
            } else if(gamepad1.dpad_right){
                points[1] = new Pose2D(0, 12, 0);
                points[2] = new Pose2D(0, 24, 0);
                dir = "right";
            } else if(gamepad1.dpad_left){
                points[1] = new Pose2D(0, -12, 0);
                points[2] = new Pose2D(0, -24, 0);
                dir = "left";
            }

            telemetry.addLine(dir);
            telemetry.update();
        }

        Path path = new Path(start, points);
        follower = new PathFollow(start, 8, path);
        drive.setFollower(follower);

        while (drive.getFollower() != null && opModeIsActive()){
            drive.update();

            telemetry.addData("Powers", drive.getPowers());
            telemetry.addLine();
            telemetry.addData("Position", drive.getPoseString());
            telemetry.addLine("Waypoint #" + (drive.getFollower().getWayPoint() + 2));

            telemetry.addLine(points[drive.getFollower().getWayPoint()+1].x + ", " +
                    points[drive.getFollower().getWayPoint()+1].y + ", " +
                    points[drive.getFollower().getWayPoint()+1].h);
            telemetry.update();
        }

        while(opModeIsActive()){
            drive.update();
            telemetry.addLine(drive.getPoseString());
            telemetry.update();
        }
    }
}
