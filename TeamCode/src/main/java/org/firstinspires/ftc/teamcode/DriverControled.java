package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "drive")
public class DriverControled extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DriveWheels drive = new DriveWheels(hardwareMap);
        EncoderWheels encoder = new EncoderWheels(hardwareMap);

        waitForStart();


        while (!isStopRequested()) {

            drive.setMotorPowers(
                    gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x
            );

            telemetry.addData("encoders",encoder);
            telemetry.update();

        }
    }
}