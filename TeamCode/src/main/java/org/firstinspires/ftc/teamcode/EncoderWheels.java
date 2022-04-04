package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class EncoderWheels {
    private DcMotor LeftTrackingWheel;
    private DcMotor RightTrackingWheel;
    private DcMotor FrontTrackingWheel;

    private double X = 0;
    private double Y = 0;
    private double Heading = 0;

    public double Left = 0;
    public double Right = 0;
    public double Front = 0;

    private double LastLeft = 0;
    private double LastRight = 0;
    private double LastFront = 0;


    public EncoderWheels(HardwareMap hardwareMap) {
        LeftTrackingWheel = hardwareMap.get(DcMotor.class, "2");
        RightTrackingWheel = hardwareMap.get(DcMotor.class, "0");
        FrontTrackingWheel = hardwareMap.get(DcMotor.class, "1");
    }

    public String toString() {
        double l = LeftTrackingWheel.getCurrentPosition();
        double r = RightTrackingWheel.getCurrentPosition();
        double f = FrontTrackingWheel.getCurrentPosition();
        return "Left: " + l + " Right: " + r + " Front: " + f;
    }

    public void setPosition(double x, double y, double heading) {
        X = x;
        Y = y;
        Front = heading;
    }

    public void updatePosition() {
        double l = LeftTrackingWheel.getCurrentPosition();
        double r = RightTrackingWheel.getCurrentPosition();
        double f = FrontTrackingWheel.getCurrentPosition();

        double LeftDiffrence = l - LastLeft;
        double RightDiffrence = r - LastRight;
        double FrontDiffrence = f - LastFront;

        LastLeft = l;
        LastRight = r;
        LastFront = f;

        Left += LeftDiffrence;
        Right += RightDiffrence;
        Front += FrontDiffrence;
    }
}
