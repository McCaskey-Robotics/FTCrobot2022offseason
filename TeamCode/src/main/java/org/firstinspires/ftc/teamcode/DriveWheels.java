package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class DriveWheels {
    private DcMotorEx leftFront;
    private DcMotorEx rightFront;
    private DcMotorEx leftBack;
    private DcMotorEx rightBack;


    public DriveWheels(HardwareMap hardwareMap) {
        leftFront = hardwareMap.get(DcMotorEx.class, "0");
        rightFront = hardwareMap.get(DcMotorEx.class, "1");
        leftBack = hardwareMap.get(DcMotorEx.class, "3");
        rightBack = hardwareMap.get(DcMotorEx.class, "2");
    }

    public void setMotorPowers(double LY, double LX, double RX){
        double r = Math.hypot(LX, LY);
        double robotAngle = Math.atan2(LY, -LX) - Math.PI / 4;

        double v1 = r * Math.cos(robotAngle) - RX;
        double v2 = r * Math.sin(robotAngle) + RX;
        double v3 = r * Math.sin(robotAngle) - RX;
        double v4 = r * Math.cos(robotAngle) + RX;

        double max = Math.max(Math.max(v1,v2),Math.max(v3,v4));

        if(max > 1){
            leftFront.setPower(v1 / max);
            rightFront.setPower(v2 / max);
            leftBack.setPower(v3 / max);
            rightBack.setPower(v4 / max);
        }
        else{
            leftFront.setPower(v1);
            rightFront.setPower(v2);
            leftBack.setPower(v3);
            rightBack.setPower(v4);
        }
    }
    public void stopMotors(){
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }
}
