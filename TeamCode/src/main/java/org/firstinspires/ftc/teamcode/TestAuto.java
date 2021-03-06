package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pathfollower.DriveWheels;
import org.firstinspires.ftc.teamcode.pathfollower.EncoderWheels;
import org.firstinspires.ftc.teamcode.pathfollower.Path;
import org.firstinspires.ftc.teamcode.pathfollower.PathFollower;
import org.firstinspires.ftc.teamcode.pathfollower.Pose2d;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@Disabled
public class TestAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DriveWheels drive = new DriveWheels(hardwareMap);
        EncoderWheels encoder = new EncoderWheels(hardwareMap);
        PathFollower pathFollower = new PathFollower(drive,encoder);

        Path path1 = new Path(new Pose2d(0,0,0),new Pose2d(10,0,Math.toRadians(180)));
        Path path2 = new Path(new Pose2d(10,0,Math.toRadians(180)),new Pose2d(0,0,0));

        pathFollower.setMaxVelocity(0.1);
        pathFollower.setAccelerationDistance(5);

        encoder.setPosition(new Pose2d());
        pathFollower.update();

        waitForStart();

        if (opModeIsActive()) {
            pathFollower.FollowPath(path1);
            pathFollower.FollowPath(path2);

        }
    }
}