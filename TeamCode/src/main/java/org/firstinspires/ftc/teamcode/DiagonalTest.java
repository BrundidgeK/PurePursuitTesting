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
            new Pose2D(0, 12, Math.toRadians(0)),
            new Pose2D(0, 24, Math.toRadians(0))
    };
    private Pose2D[] points2 = new Pose2D[] {
            new Pose2D(0, 24, Math.toRadians(0)),
            new Pose2D(0, 12, Math.toRadians(0)),
            new Pose2D(0, 0,Math.toRadians(0))
    };

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2D start = new Pose2D(0,0,0);
        Path path1 = new Path(start, points1);

        follower1 = new PathFollow(start, 8, path1);

        drive = new MecDrivebase(hardwareMap, start, .5);
        double time = 0;

        waitForStart();

        while (!concludePath(path1.getPt(points1.length-1)) && opModeIsActive()){
            drive.update();

            Pose2D move = follower1.followPath(drive.getPose());
            if(!Double.isNaN(move.x)) {
                telemetry.addLine("moving normally");
                telemetry.addLine();
                drive.moveTo(move.x, move.y, path1.getPt(follower1.getWayPoint()).h-drive.getPose().h);
            }
            else{
                telemetry.addLine("moving differently");
                telemetry.addLine();
                Pose2D tar = path1.getPt(follower1.getWayPoint());
                Pose2D diff = new Pose2D(tar.x-drive.getPose().x,
                        tar.y-drive.getPose().y,
                        tar.h-drive.getPose().h);
                drive.moveTo(diff);
            }

            telemetry.addLine(drive.getPowers());
            telemetry.addLine(""+move.x);
            telemetry.addLine(""+move.y);
            telemetry.addLine(""+move.h);

            telemetry.addData("Position", drive.getPoseString());
            telemetry.addLine("Waypoint #" + (follower1.getWayPoint() + 2));


            telemetry.addLine(points1[follower1.getWayPoint()+1].x + ", " +
                    points1[follower1.getWayPoint()+1].y + ", " +
                    points1[follower1.getWayPoint()+1].h);
            telemetry.update();
        }

        drive.moveWithPower(0);
        sleep(5000);
        Path path2 = new Path(drive.getPose(), points2);
        follower2 = new PathFollow(drive.getPose(), 8, path2);

        while (!concludePath(path2.getPt(points2.length-1)) && opModeIsActive()){
            drive.update();

            Pose2D move = follower2.followPath(drive.getPose());
            if(!Double.isNaN(move.x)) {
                telemetry.addLine("moving normally");
                telemetry.addLine();
                drive.moveTo(move);
            }
            else{
                telemetry.addLine("moving differently");
                telemetry.addLine();
                Pose2D tar = path2.getPt(follower2.getWayPoint());
                Pose2D diff = new Pose2D(tar.x-drive.getPose().x,
                        tar.y-drive.getPose().y,
                        tar.h-drive.getPose().h);
                drive.moveTo(diff);
            }

            telemetry.addLine(drive.getPowers());
            telemetry.addLine(""+move.x);
            telemetry.addLine(""+move.y);
            telemetry.addLine(""+move.h);

            telemetry.addData("Position", drive.getPoseString());
            telemetry.addLine("Waypoint #" + (follower2.getWayPoint() + 2));


            telemetry.addLine(points2[follower2.getWayPoint()+1].x + ", " +
                    points2[follower2.getWayPoint()+1].y + ", " +
                    points2[follower2.getWayPoint()+1].h);
            telemetry.update();
        }
    }

    private boolean concludePath(Pose2D target){
        return distance(target, drive.getPose()) <= 2.5;//&& follower.isShrinkingLook();
    }

    private boolean turning(double tarAngle){
        return  Math.abs(tarAngle - drive.getPose().h) <= Math.toRadians(5);
    }

    private double distance(Pose2D a, Pose2D b){
        return Math.hypot(a.x-b.x, a.y-b.y);
    }
}
