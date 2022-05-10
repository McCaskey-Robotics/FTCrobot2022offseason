package org.firstinspires.ftc.teamcode;

import android.drm.DrmStore;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;

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
    DigitalChannel leftLED_G;
    DigitalChannel leftLED_R;
    DigitalChannel rightLED_G;
    DigitalChannel rightLED_R;


    double lastX = 0;
    double lastY = 0;
    double lasth = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        DriveWheels drive = new DriveWheels(hardwareMap);
        pivotPoint = hardwareMap.get(DcMotorSimple.class,"pivotPoint");
        leftLED_G = hardwareMap.get(DigitalChannel.class,"LG");
        leftLED_R = hardwareMap.get(DigitalChannel.class,"LR");
        rightLED_G = hardwareMap.get(DigitalChannel.class,"RG");
        rightLED_R = hardwareMap.get(DigitalChannel.class,"RR");

        leftLED_G .setMode(DigitalChannel.Mode.OUTPUT);
        leftLED_R .setMode(DigitalChannel.Mode.OUTPUT);
        rightLED_G .setMode(DigitalChannel.Mode.OUTPUT);
        rightLED_R .setMode(DigitalChannel.Mode.OUTPUT);


        waitForStart();

        while (!isStopRequested()) {

            drive.setMotorPowers(
                    gamepad1.left_stick_y,
                    gamepad1.left_stick_x / 2,
                    gamepad1.right_stick_x / 5);

            pivotPoint.setPower(gamepad1.left_trigger - gamepad1.right_trigger);

            telemetry.addData("y, ",gamepad1.left_stick_y + " " + lastY);
            telemetry.addData("x, ",gamepad1.left_stick_x);
            telemetry.addData("h, ",gamepad1.right_stick_x);
            telemetry.update();

            //off
            if(gamepad1.dpad_left){
                leftLED_G.setState(true);
                leftLED_R.setState(true);
                rightLED_G.setState(true);
                rightLED_R.setState(true);
            }
            //green
            else if(gamepad1.dpad_down){
                leftLED_G.setState(true);
                leftLED_R.setState(false);
                rightLED_G.setState(true);
                rightLED_R.setState(false);
            }
            //red
            else if(gamepad1.dpad_right){
                leftLED_G.setState(false);
                leftLED_R.setState(true);
                rightLED_G.setState(false);
                rightLED_R.setState(true);
            }
            //amber
            else if(gamepad1.dpad_up){
                leftLED_G.setState(false);
                leftLED_R.setState(false);
                rightLED_G.setState(false);
                rightLED_R.setState(false);
            }
            /*
            leftLED_G.setState(gamepad1.dpad_up);
            leftLED_R.setState(gamepad1.dpad_down);
            rightLED_G.setState(gamepad1.dpad_left);
            rightLED_R.setState(gamepad1.dpad_right);
             */

        }
    }
}