package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

@Config
public class PathFollower {
    DriveWheels driveWheels;
    EncoderWheels encoderWheels;
    Path currentpath;

    FtcDashboard dashboard = FtcDashboard.getInstance();

    //0 to 1
    double maxVelocity = 1;

    //distance to start slowing down
    double accelerationDistance = 15;

    double theta = 0;

    //allowed error
    double allowedError = 2;

    enum state {
        Following,
        Idle,
    }

    state currentState = state.Idle;

    public PathFollower(DriveWheels driveWheels,EncoderWheels encoderWheels) {
        this.driveWheels = driveWheels;
        this.encoderWheels = encoderWheels;

        dashboard.setTelemetryTransmissionInterval(25);
    }

    //follows path
    public void FollowPathAsync(Path currentpath){
        this.currentpath = currentpath;
        currentState = state.Following;
    }

    //follows path
    public void FollowPath(Path currentpath){
        FollowPathAsync(currentpath);
        while (!Thread.currentThread().isInterrupted() && isBusy()) {
            update();
            drawPath();
        }
    }

    //return if currently following a path
    public boolean isBusy(){
        return currentState == state.Following;
    }

    //cancel the following
    public void cancleFollowing(){
        currentState = state.Idle;
    }

    //update wheel speeds must be called in a loop while following a path from zachPathGenerator
    public void update(){

        encoderWheels.updatePosition();

        if(currentState == state.Following){
            double power = 1;

            double DistanceToTarget = getDistance(encoderWheels.getCurrentPosition(), currentpath.endPose);

            //if we are close to target start reducing power
            if(DistanceToTarget < accelerationDistance){
                //start slowing down always keep power over 0.1
                //graph I made: motorPower = y, DistanceToTarget = x
                //https://www.desmos.com/calculator/9qcrvbhcsn
                power = ((0.9 / accelerationDistance) * DistanceToTarget) + 0.1;

            }
            power *= maxVelocity;

            updateTheta();

            double theta2 = theta - encoderWheels.heading;

            double x = Math.cos(theta2 - Math.PI / 2) * -power;
            double y = Math.sin(theta2 - Math.PI / 2) * power;

            double headingDiffrence = getHeadingDiffrence();

            double h = 0;

            //if we have reached our target position
            if(DistanceToTarget > allowedError) {
                //set the motor powers
                driveWheels.setMotorPowers(y, x, h);
            }
            else{
                //stop motors
                driveWheels.stopMotors();

                //if we are not moving change state to idle
                if(encoderWheels.getAverageVelocity() == 0){
                    currentState = state.Idle;
                }
            }

            //drive at power towards the target while turning towards the target heading
        }
        else if(currentState == state.Idle){
            //stop motors
            driveWheels.stopMotors();
        }
    }

    public void updateTheta() {
        theta = Math.atan2(currentpath.getEndPose().getY() - encoderWheels.getCurrentPosition().getY(),
                currentpath.getEndPose().getX() - encoderWheels.getCurrentPosition().getX());
    }

    public Path getPath() {
        return currentpath;
    }

    public void setMaxVelocity(double maxVelocity) {
        this.maxVelocity = maxVelocity;
    }

    public double getMaxVelocity() {
        return maxVelocity;
    }

    public void setAccelerationDistance(double accelerationDistance) {
        this.accelerationDistance = accelerationDistance;
    }

    public double getAccelerationDistance() {
        return accelerationDistance;
    }

    //uses pythagorean theorem to find distance between two pose2d
    public double getDistance(Pose2d pose1, Pose2d pose2){
        return Math.sqrt(Math.pow(pose1.getX() - pose2.getX(), 2) + Math.pow(pose1.getY() - pose2.getY(), 2));
    }

    public double getHeadingDiffrence(){
        return(getPath().endPose.heading - getPath().startPose.heading);
    }

    public double getPercent(){
        double DistanceToTarget = getDistance(encoderWheels.getCurrentPosition(), currentpath.endPose);
        double PathDistance = getDistance(currentpath.startPose, currentpath.endPose);
        return 0; //TODO
    }

    public void drawPath() {

        double x_pos1 = currentpath.startPose.getX();
        double y_pos1 = currentpath.startPose.getY();
        double heading1 = currentpath.startPose.getHeading();

        double x_pos2 = currentpath.endPose.getX();
        double y_pos2 = currentpath.endPose.getY();
        double heading2 = currentpath.endPose.getHeading();

        TelemetryPacket packet = new TelemetryPacket();

        packet.addLine("X: " + encoderWheels.getCurrentPosition().getX());
        packet.addLine("Y: " + encoderWheels.getCurrentPosition().getY());
        packet.addLine("Heading: " + encoderWheels.getCurrentPosition().getHeading());

        packet.fieldOverlay()
                //start position
                .setStroke("#0000ff")
                .setStrokeWidth(1)
                .strokeCircle(x_pos1,  y_pos1, 9)
                .strokeLine(x_pos1,y_pos1,x_pos1 + Math.cos(heading1) * 10,y_pos1 + Math.sin(heading1) * 10)

                //end position
                .strokeCircle(x_pos2,  y_pos2, 9)
                .strokeLine(x_pos2,y_pos2,x_pos2 + Math.cos(heading2) * 10,y_pos2 + Math.sin(heading2) * 10)

                //path
                .strokeLine(x_pos1,y_pos1,x_pos2,y_pos2)

                //robot
                .setStroke("#00ff00")
                .strokeCircle(encoderWheels.x_pos,  encoderWheels.y_pos, 9)
                .strokeLine(encoderWheels.x_pos,encoderWheels.y_pos,encoderWheels.x_pos + Math.cos(encoderWheels.heading) * 10,encoderWheels.y_pos + Math.sin(encoderWheels.heading) * 10)
                .setStroke("#ff0000")
                .strokeLine(encoderWheels.x_pos,encoderWheels.y_pos,encoderWheels.x_pos + Math.cos(theta) * 10,encoderWheels.y_pos + Math.sin(theta) * 10);



        dashboard.sendTelemetryPacket(packet);
    }

}
