package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import Wheelie.Path;
import Wheelie.Pose2D;

@Autonomous
public class ComboTest extends LinearOpMode {
    private MecDrivebase drive;
    private PathFollow follower1, follower2;

    private Pose2D[] points1 = new Pose2D[]{
            new Pose2D(0, 0, 0),
            new Pose2D(24, 0, Math.toRadians(0)),
            new Pose2D(24, 24, Math.toRadians(90))
    };

    private Pose2D[] points2 = new Pose2D[]{
            new Pose2D(24, 24, Math.toRadians(90)),
            new Pose2D(24, 0, Math.toRadians(0)),
            new Pose2D(0, 0, Math.toRadians(0))
    };

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2D start = new Pose2D(0, 0, 0);
        Path path1 = new Path(start, points1),
            path2 = new Path(start, points2);

        follower1 = new PathFollow(start, 8, path1);

        drive = new MecDrivebase(hardwareMap, start, 1);

        waitForStart();

        drive.setFollower(follower1);

        while (!drive.targetReached(path1.getPt(points1.length - 1)) && opModeIsActive()) {
            drive.update(9);


            telemetry.addData("Position", drive.getPoseString());
            telemetry.addLine("Waypoint #" + (follower1.getWayPoint() + 2));

            telemetry.addLine(points1[follower1.getWayPoint() + 1].x + ", " +
                    points1[follower1.getWayPoint() + 1].y + ", " +
                    points1[follower1.getWayPoint() + 1].h);

            telemetry.addLine();
            telemetry.addLine(drive.getPowers());

            telemetry.update();
        }

        drive.setFollower(follower2);

        follower2 = new PathFollow(drive.getPose(), 8, path2);
        sleep(5000);

        while (!drive.targetReached(path2.getPt(points2.length - 1)) && opModeIsActive()) {
            drive.update(9);

            telemetry.addData("Position", drive.getPoseString());
            telemetry.addLine("Waypoint #" + (follower2.getWayPoint() + 2));


            telemetry.addLine(points1[follower2.getWayPoint() + 1].x + ", " +
                    points1[follower2.getWayPoint() + 1].y + ", " +
                    points1[follower2.getWayPoint() + 1].h);

            telemetry.addLine();
            telemetry.addLine(drive.getPowers());

            telemetry.update();
        }
    }

    private boolean concludePath(Pose2D target) {
        return distance(target, drive.getPose()) <= 2 && Math.abs(target.h - drive.getPose().h) < Math.toRadians(5);//&& follower.isShrinkingLook();
    }

    private double distance(Pose2D a, Pose2D b) {
        return Math.hypot(a.x - b.x, a.y - b.y);
    }
}
