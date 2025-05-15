package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.ArmSubsystems.*;

public class OverrideCmd extends Command{

    private SwerveSubsystem swerveSubsystem;
    private IntakeMotorSubsystem intakeSubsystem;
    private PitchMotorSubsystem pitchSubsystem;
    private ShootingMotorSubsystem shooterSubsystem;
    
    public OverrideCmd(SwerveSubsystem swerveSubsystem, IntakeMotorSubsystem intakeSubsystem, PitchMotorSubsystem pitchSubsystem, ShootingMotorSubsystem shooterSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.pitchSubsystem = pitchSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(intakeSubsystem);
        addRequirements(swerveSubsystem); 
        addRequirements(pitchSubsystem);
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return true;
    }

}
