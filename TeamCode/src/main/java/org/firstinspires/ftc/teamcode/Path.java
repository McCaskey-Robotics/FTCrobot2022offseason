package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Path {

    Pose2d startPose;
    Pose2d endPose;

    public Path(Pose2d startPose, Pose2d endPose) {
        this.startPose = startPose;
        this.endPose = endPose;
    }

    public Path() {
        startPose = new Pose2d(0, 0, 0);
        endPose = new Pose2d(0, 0, 0);
    }

    public Pose2d getStartPose() {
        return startPose;
    }
    public Pose2d getEndPose() {
        return endPose;
    }

    public void setStartPose(Pose2d startPose) {
        this.startPose = startPose;
    }
    public void setEndPose(Pose2d endPose) {
        this.endPose = endPose;
    }
}
