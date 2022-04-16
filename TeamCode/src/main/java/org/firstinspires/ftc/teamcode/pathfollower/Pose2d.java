package org.firstinspires.ftc.teamcode.pathfollower;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Pose2d {
    double x,y,heading;

    public Pose2d(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public Pose2d() {
        this.x = 0;
        this.y = 0;
        this.heading = 0;
    }

    public Pose2d(Pose2d p) {
        this.x = p.x;
        this.y = p.y;
        this.heading = p.heading;
    }
    public double getX() {
        return x;
    }
    public double getY() {
        return y;
    }
    public double getHeading() {
        return heading;
    }
    public void setX(double x) {
        this.x = x;
    }
    public void setY(double y) {
        this.y = y;
    }
    public void setHeading(double heading) {
        this.heading = heading;
    }
}
