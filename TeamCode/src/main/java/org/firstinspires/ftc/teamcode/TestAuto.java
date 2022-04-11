package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@Autonomous()
public class TestAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DriveWheels drive = new DriveWheels(hardwareMap);
        EncoderWheels encoder = new EncoderWheels(hardwareMap);
        PathFollower pathFollower = new PathFollower(drive,encoder);

        Path path1 = new Path(new Pose2d(0,0,0),new Pose2d(20,20,Math.toRadians(90)));
        Path path2 = new Path(path1.getEndPose(), new Pose2d(20,-20,0));
        Path path3 = new Path(path2.getEndPose(), path1.getStartPose());

        pathFollower.setMaxVelocity(0.5);
        pathFollower.setAccelerationDistance(5);

        waitForStart();

        if (opModeIsActive()) {
            pathFollower.FollowPath(path1);

        }
    }
}