package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import Wheelie.Path;
import Wheelie.Pose2D;

@Autonomous
public class TurnTest extends LinearOpMode {
    private MecDrivebase drive;
    private PathFollow follower;

    private Pose2D[] points = new Pose2D[] {
            new Pose2D(0, 0,0),
            new Pose2D(0, 0, Math.toRadians(45)),
            new Pose2D(0, 0, Math.toRadians(90)),
    };

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2D start = new Pose2D(0,0,0);

        drive = new MecDrivebase(hardwareMap, start);

        waitForStart();

        Path path = new Path(start, points);
        follower = new PathFollow(start, 8, path);
        drive.setFollower(follower);

        while (drive.getFollower() != null && opModeIsActive()){
            drive.update(1);

            telemetry.addData("Powers", drive.getPowers());
            telemetry.addLine();
            telemetry.addData("Position", drive.getPoseString());
            telemetry.addData("Movement", drive.m.x);
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

    private boolean concludePath(Pose2D target){
        return distance(target, drive.getPose()) <= 2 && Math.abs(target.h-drive.getPose().h) < Math.toRadians(5);//&& follower.isShrinkingLook();
    }

    private double distance(Pose2D a, Pose2D b){
        return Math.hypot(a.x-b.x, a.y-b.y);
    }
}
