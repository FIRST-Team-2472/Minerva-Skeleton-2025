package frc.robot;

public class AimPoint {

    private double yawAngle, pitchAngle;

    AimPoint(double yawAngle, double pitchAngle){
        this.yawAngle = yawAngle;
        this.pitchAngle = pitchAngle;
    }

    public double getPitchAngle(){
        return pitchAngle;
    }

    public double getYawAngle(){
        return yawAngle;
    }
}