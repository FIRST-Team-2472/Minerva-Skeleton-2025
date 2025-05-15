package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.ArmSubsystems.IntakeMotorSubsystem;
import frc.robot.subsystems.ArmSubsystems.PitchMotorSubsystem;
import frc.robot.subsystems.ArmSubsystems.ShootingMotorSubsystem;

public class ConstantAimToggleCmd extends Command{

    SwerveSubsystem swerveSubsystem;
    PitchMotorSubsystem pitchMotorSubsystem;
    ShootingMotorSubsystem shootingMotorSubsystem;
    
    /**
     *  a toggle button that changes some booleans which cause
     * the arm, and the robot to angle to the speaker, rev up the shooter motors and instantly ends
     *  warning, does cancel any current commands with those subsystems
     * @param swerveSubsystem the swerve subsystem
     * @param pitchMotorSubsystem the pitch motor subsytem
     * @param shootingMotorSubsystem the shooting motor subsystem
     */

    public ConstantAimToggleCmd(SwerveSubsystem swerveSubsystem, PitchMotorSubsystem pitchMotorSubsystem,
     ShootingMotorSubsystem shootingMotorSubsystem){
        this.swerveSubsystem = swerveSubsystem;
        this.pitchMotorSubsystem = pitchMotorSubsystem;
        this.shootingMotorSubsystem = shootingMotorSubsystem;

        addRequirements(swerveSubsystem);
        addRequirements(pitchMotorSubsystem);
        addRequirements(shootingMotorSubsystem);
    }

    @Override
    public void initialize(){
        shootingMotorSubsystem.constantAim();
        pitchMotorSubsystem.constantAim();
        swerveSubsystem.constantAim();
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
