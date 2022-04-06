package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class PathFollower {
    DriveWheels driveWheels;
    EncoderWheels encoderWheels;
    Path currentpath;

    //0 to 1
    double maxVelocity = 0.5;

    //distance to start slowing down
    double accelerationDistance = 10;


    enum state {
        Following,
        Idle,
    }

    state currentState = state.Idle;

    public PathFollower(DriveWheels driveWheels,EncoderWheels encoderWheels) {
        this.driveWheels = driveWheels;
        this.encoderWheels = encoderWheels;
    }

    //follows path
    public void FollowPathAsync(Path currentpath){
        this.currentpath = currentpath;
        currentState = state.Following;
    }

    //follows path
    public void FollowPath(Path currentpath){
        FollowPathAsync(currentpath);
        while (!Thread.currentThread().isInterrupted() && isBusy())
            update();
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

            //drive at power towards the target while turning towards the target heading
        }
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

}
