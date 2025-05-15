package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AutoAiming;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.ArmSubsystems.PitchMotorSubsystem;

public class SetArmPitchCmd extends Command {
    private PitchMotorSubsystem pitchMotorSubsystem;
    SwerveSubsystem swerveSubsystem;
    private double angleDeg;
    private final Timer timer;
    private boolean autoAiming;

    public SetArmPitchCmd(PitchMotorSubsystem pitchMotorSubsystem, double angleDeg) {
        this.angleDeg = angleDeg;
        this.timer = new Timer();
        this.pitchMotorSubsystem = pitchMotorSubsystem;
        addRequirements(pitchMotorSubsystem);
    }
    
    public SetArmPitchCmd(PitchMotorSubsystem pitchMotorSubsystem, SwerveSubsystem swerveSubsystem) {
        this(pitchMotorSubsystem, 0.0);
        autoAiming = true;
        this.swerveSubsystem = swerveSubsystem;
    }   

    @Override
    public void initialize() {
        timer.stop();
        timer.reset();
        timer.start();
        if(autoAiming)
            angleDeg = AutoAiming.getPitch(swerveSubsystem.getPose());
    }

    @Override
    public void execute() {
        pitchMotorSubsystem.runPitchMotorWithFasterKP(angleDeg);
    }

    @Override
    public void end(boolean interrupted) {
        pitchMotorSubsystem.runPitchMotor(0.0);
    }

    @Override
    public boolean isFinished() {
        return (Math.abs(pitchMotorSubsystem.getEncoderDeg() - angleDeg) < 0.5) || (timer.hasElapsed(3));
    }

}
