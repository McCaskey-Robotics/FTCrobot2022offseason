package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class PathFollower {
    Path path;

    enum states {
        DRIVING,
        FINISHED
    }

    states state = states.DRIVING;

    public void update(double x, double y, double theta){
        switch(state){
            case DRIVING:
                if(path.isFinished()){
                    state = states.FINISHED;
                }
                break;
            case FINISHED:
                break;
        }
    }

    public PathFollower() {
        this.path = new Path();
    }

    public PathFollower(Path path) {
        this.path = path;
    }

    public void setPath(Path path) {
        this.path = path;
    }

    public void update(){

    }
}
