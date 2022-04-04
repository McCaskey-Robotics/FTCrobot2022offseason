package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class DriveWheels {
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;


    public DriveWheels(HardwareMap hardwareMap) {
        leftFront = hardwareMap.get(DcMotor.class, "0");
        rightFront = hardwareMap.get(DcMotor.class, "1");
        leftBack = hardwareMap.get(DcMotor.class, "3");
        rightBack = hardwareMap.get(DcMotor.class, "2");
    }

    public void setMotorPowers(double LY, double LX, double RX){
        double r = Math.hypot(LX, LY);
        double robotAngle = Math.atan2(LY, -LX) - Math.PI / 4;

        double v1 = r * Math.cos(robotAngle) - RX;
        double v2 = r * Math.sin(robotAngle) + RX;
        double v3 = r * Math.sin(robotAngle) - RX;
        double v4 = r * Math.cos(robotAngle) + RX;

        leftFront.setPower(v1);
        rightFront.setPower(v2);
        leftBack.setPower(v3);
        rightBack.setPower(v4);
    }
}