package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveRotateToAngle extends Command {
  private SwerveSubsystem swerveSubsystem;
  private Rotation2d targetAngle;
  private Timer timer, override;
  private boolean autoAiming;

  public SwerveRotateToAngle(SwerveSubsystem m_SwerveSubsystem, Rotation2d targetAngle) {
    this.swerveSubsystem = m_SwerveSubsystem;
    this.targetAngle = targetAngle;
    
    timer = new Timer();
    override = new Timer();

    addRequirements(m_SwerveSubsystem);
  }
  public SwerveRotateToAngle(SwerveSubsystem m_SwerveSubsystem) {
    this(m_SwerveSubsystem, new Rotation2d());
    autoAiming = true;
  }

  @Override
  public void initialize() {
    swerveSubsystem.initializeRotateToAngle();
    timer.restart();
    override.restart();
  }

  @Override
  public void execute() {
    swerveSubsystem.executeRotateToAngle(targetAngle);
  }

  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stopModules();
  }

  @Override
  public boolean isFinished() {
    if (override.hasElapsed(3))
      return true;
    
    // use this function if you override the command to finish it
    if ( swerveSubsystem.isAtAngle(targetAngle))
      return timer.hasElapsed(.2);
    timer.restart();
    return false;
  }
}
