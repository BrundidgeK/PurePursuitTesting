package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import Wheelie.Path;
import Wheelie.Pose2D;

@Autonomous
public class TestPursuit2 extends LinearOpMode {
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
        double time = 0;

        waitForStart();

        while (!concludePath(path.getPt(points.length-1)) && opModeIsActive()){
            drive.update();


            Pose2D move = follower.followPath(drive.getPose());
            if (path.getPt(follower.getWayPoint()) != path.getPt(path.pathLength()-1)){
                telemetry.addLine("moving normally");
                telemetry.addLine(drive.getPowers());
                drive.moveTo(move);
                telemetry.addLine(""+move.x);
                telemetry.addLine(""+move.y);
                telemetry.addLine(""+move.h);
            } else {
                telemetry.addLine("Using PID");
                if(time == 0)
                    time = System.currentTimeMillis();
                drive.moveToPID(move, path.getPt(points.length-1), time);
            }

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
