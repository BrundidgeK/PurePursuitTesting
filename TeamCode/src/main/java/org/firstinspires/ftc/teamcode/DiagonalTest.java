package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import Wheelie.Path;
import Wheelie.Pose2D;

@Autonomous
public class DiagonalTest extends LinearOpMode {
    private MecDrivebase drive;
    private PathFollow follower1, follower2;

    private Pose2D[] points1 = new Pose2D[] {
            new Pose2D(0, 0,Math.toRadians(0)),
            new Pose2D(12, 12, Math.toRadians(0)),
            new Pose2D(24, 24, Math.toRadians(0))
    };
    private Pose2D[] points2 = new Pose2D[] {
            new Pose2D(24, 24, Math.toRadians(0)),
            new Pose2D(12, 12, Math.toRadians(0)),
            new Pose2D(0, 0,Math.toRadians(0))
    };

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2D start = new Pose2D(0,0,0);
        Path path1 = new Path(start, points1);

        follower1 = new PathFollow(start, 8, path1);

        drive = new MecDrivebase(hardwareMap, start, .5);
        waitForStart();

        drive.setFollower(follower1);

        while (drive.getFollower() != null && opModeIsActive()){
            drive.update();

            telemetry.addData("Powers", drive.getPowers());
            telemetry.addLine();
            telemetry.addData("Position", drive.getPoseString());
            telemetry.addLine("Waypoint #" + (drive.getFollower().getWayPoint() + 2));

            telemetry.addLine(points1[drive.getFollower().getWayPoint()+1].x + ", " +
                    points1[drive.getFollower().getWayPoint()+1].y + ", " +
                    points1[drive.getFollower().getWayPoint()+1].h);
            telemetry.update();
        }

        sleep(5000);

        Path path2 = new Path(drive.getPose(), points2);
        follower2 = new PathFollow(drive.getPose(), 8, path2);
        drive.setFollower(follower2);

        while (drive.getFollower() != null && opModeIsActive()){
            drive.update();

            telemetry.addData("Powers", drive.getPowers());
            telemetry.addLine();
            telemetry.addData("Position", drive.getPoseString());
            telemetry.addLine("Waypoint #" + (drive.getFollower().getWayPoint() + 2));

            telemetry.addLine(points2[drive.getFollower().getWayPoint()+1].x + ", " +
                    points2[drive.getFollower().getWayPoint()+1].y + ", " +
                    points2[drive.getFollower().getWayPoint()+1].h);
            telemetry.update();
        }

        while(opModeIsActive()){
            drive.update();
            telemetry.addLine(drive.getPoseString());
            telemetry.update();
        }
    }
}
