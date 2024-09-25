package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import Wheelie.Path;
import Wheelie.Pose2D;

@Autonomous
public class TestPursuit3 extends LinearOpMode {
    private MecDrivebase drive;
    private PathFollow follower;

    private Pose2D[] points = new Pose2D[] {
            new Pose2D(0, 0,0),
            new Pose2D(24, 0, 0),
            new Pose2D(24, 24, 0),
    };

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2D start = new Pose2D(0,0,0);
        Path path = new Path(start, points);

        follower = new PathFollow(start, 3, path);

        drive = new MecDrivebase(hardwareMap, start);


        waitForStart();
        int currentWay = 0;
        double time = System.currentTimeMillis();

        while (!concludePath(path.getPt(points.length-1)) && opModeIsActive()){
            drive.update();

            if(currentWay != follower.getWayPoint()){
                time = System.currentTimeMillis();
                currentWay = follower.getWayPoint();
            }


            telemetry.addLine(drive.getPowers());
            Pose2D move = follower.followPath(drive.getPose());
            drive.moveToPID(move, path.getPt(currentWay), time);

                telemetry.addLine(""+move.x);
                telemetry.addLine(""+move.y);
                telemetry.addLine(""+move.h);


            telemetry.addData("Position", drive.getPoseString());
            telemetry.addLine("Waypoint #" + (follower.getWayPoint() + 2));


            telemetry.addLine(points[follower.getWayPoint()+1].x + ", " +
                    points[follower.getWayPoint()+1].y + ", " +
                    points[follower.getWayPoint()+1].h);
            telemetry.update();
        }
    }

    private boolean concludePath(Pose2D target){
        return distance(target, drive.getPose()) <= 2 ;//&& follower.isShrinkingLook();
    }

    private double distance(Pose2D a, Pose2D b){
        return Math.hypot(a.x-b.x, a.y-b.y);
    }
}
