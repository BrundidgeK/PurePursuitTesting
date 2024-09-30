package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import Wheelie.Path;
import Wheelie.Pose2D;

@Autonomous
public class ComboTest extends LinearOpMode {
    private MecDrivebase drive;
    private PathFollow follower;

    private Pose2D[] points = new Pose2D[] {
            new Pose2D(0, 0,0),
            new Pose2D(24, 0, Math.toRadians(45)),
            new Pose2D(24, 24, Math.toRadians(-90)),
    };

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2D start = new Pose2D(0,0,0);
        Path path = new Path(start, points);

        follower = new PathFollow(start, 8, path);

        drive = new MecDrivebase(hardwareMap, start);

        waitForStart();

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
