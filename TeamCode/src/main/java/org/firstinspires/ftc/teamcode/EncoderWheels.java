package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class EncoderWheels {
    public static double track_width = 16;
    public static double forward_offset = 4;

    private DcMotor LeftTrackingWheel;
    private DcMotor RightTrackingWheel;
    private DcMotor FrontTrackingWheel;

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
        LeftTrackingWheel = hardwareMap.get(DcMotor.class, "2");
        RightTrackingWheel = hardwareMap.get(DcMotor.class, "0");
        FrontTrackingWheel = hardwareMap.get(DcMotor.class, "1");
    }

    public String toString() {
        double l = -LeftTrackingWheel.getCurrentPosition();
        double r = RightTrackingWheel.getCurrentPosition();
        double f = FrontTrackingWheel.getCurrentPosition();
        return "Left: " + Left + "\nRight: " + Right + "\nFront: " + Front + "\nX" + x_pos + "\nY" + y_pos + "\nHeading" + heading;
    }

    public void setPosition(double x_pos, double y_pos, double heading) {
        this.x_pos = x_pos;
        this.y_pos = y_pos;
        this.heading = heading;
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
                .strokeCircle(x_pos,  y_pos, 10)
                .strokeLine(x_pos,y_pos,x_pos + Math.cos(heading) * 10,y_pos + Math.sin(heading) * 10);

        dashboard.sendTelemetryPacket(packet);
    }
}
