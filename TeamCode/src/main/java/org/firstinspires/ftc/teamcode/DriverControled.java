package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

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
@TeleOp(name = "Driver Controlled")
public class DriverControled extends LinearOpMode {
    DcMotorSimple pivotPoint;
    @Override
    public void runOpMode() throws InterruptedException {
        DriveWheels drive = new DriveWheels(hardwareMap);
        pivotPoint = hardwareMap.get(DcMotorSimple.class,"pivotPoint");

        waitForStart();

        while (!isStopRequested()) {

            drive.setMotorPowers(
                    gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x);

            pivotPoint.setPower(gamepad1.left_trigger - gamepad1.right_trigger);

        }
    }
}