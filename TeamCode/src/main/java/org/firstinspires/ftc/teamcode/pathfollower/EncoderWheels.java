package org.firstinspires.ftc.teamcode.pathfollower;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class EncoderWheels {
    public static double track_width = 16;
    public static double forward_offset = 4;

    private DcMotorEx LeftTrackingWheel;
    private DcMotorEx RightTrackingWheel;
    private DcMotorEx FrontTrackingWheel;

    double x_pos = 0;
    double y_pos = 0;
    double heading = 0;

    public double Left = 0;
    public double Right = 0;
    public double Front = 0;

    double prev_left_encoder_pos = 0;
    double prev_right_encoder_pos = 0;
    double prev_center_encoder_pos = 0;


    public EncoderWheels(HardwareMap hardwareMap) {
        LeftTrackingWheel = hardwareMap.get(DcMotorEx.class, "2");
        RightTrackingWheel = hardwareMap.get(DcMotorEx.class, "0");
        FrontTrackingWheel = hardwareMap.get(DcMotorEx.class, "1");
    }

    public String toString() {
        double l = -LeftTrackingWheel.getCurrentPosition();
        double r = RightTrackingWheel.getCurrentPosition();
        double f = FrontTrackingWheel.getCurrentPosition();
        return "Left: " + Left + "\nRight: " + Right + "\nFront: " + Front + "\nX" + x_pos + "\nY" + y_pos + "\nHeading" + heading;
    }

    public void setPosition(Pose2d pose2d) {
        x_pos = pose2d.getX();
        y_pos = pose2d.getY();
        heading = pose2d.getHeading();
    }

    public Pose2d getCurrentPosition() {
        return new Pose2d(x_pos, y_pos, heading);
    }

    public void updatePosition() {

        double left_encoder_pos = encoderPositionToInches(-LeftTrackingWheel.getCurrentPosition());
        double right_encoder_pos = encoderPositionToInches(RightTrackingWheel.getCurrentPosition());
        double center_encoder_pos = encoderPositionToInches(FrontTrackingWheel.getCurrentPosition());

        double delta_left_encoder_pos = left_encoder_pos - prev_left_encoder_pos;
        double delta_right_encoder_pos = right_encoder_pos - prev_right_encoder_pos;
        double delta_center_encoder_pos = center_encoder_pos - prev_center_encoder_pos;

        double phi = (delta_left_encoder_pos - delta_right_encoder_pos) / track_width;
        double delta_middle_pos = (delta_left_encoder_pos + delta_right_encoder_pos) / 2;
        double delta_perp_pos = delta_center_encoder_pos - forward_offset * phi;

        double delta_x = delta_middle_pos * Math.cos(heading) - delta_perp_pos * -Math.sin(heading);
        double delta_y = delta_middle_pos * -Math.sin(heading) + delta_perp_pos * Math.cos(heading);

        x_pos += delta_x;
        y_pos -= delta_y;
        heading -= phi;

        prev_left_encoder_pos = left_encoder_pos;
        prev_right_encoder_pos = right_encoder_pos;
        prev_center_encoder_pos = center_encoder_pos;
    }

    private double encoderPositionToInches(double encoderPosition) {
        return encoderPosition / 8192 * Math.PI * 1.5;
    }

    public void drawRobot(FtcDashboard dashboard) {
        TelemetryPacket packet = new TelemetryPacket();
        packet.fieldOverlay()
                .setStrokeWidth(1)
                .strokeCircle(x_pos,  y_pos, 9)
                .strokeLine(x_pos,y_pos,x_pos + Math.cos(heading) * 10,y_pos + Math.sin(heading) * 10);

        dashboard.sendTelemetryPacket(packet);
    }
    public double getAverageVelocity(){
        double v = 0;
        v += LeftTrackingWheel.getVelocity();
        v += RightTrackingWheel.getVelocity();
        v += FrontTrackingWheel.getVelocity();
        return v / 3;
    }
}
