package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.ArmSubsystems.*;

public class FastAutoAimCmd extends SequentialCommandGroup {

    public FastAutoAimCmd(PitchMotorSubsystem pitchSubsystem, SwerveSubsystem swerveSubsystem,
            ShootingMotorSubsystem shootingSubsystem, IntakeMotorSubsystem intakeSubsystem) {
        super(
                new ParallelDeadlineGroup(
                        new ParallelCommandGroup(
                                new SetArmPitchCmd(pitchSubsystem, swerveSubsystem),
                                new SwerveRotateToAngle(swerveSubsystem)
                        ),
                        new RunShooterCmd(shootingSubsystem, 4000)
                ),
                new ShootNoteCmd(shootingSubsystem, intakeSubsystem, 0.9, 4000));
    }
}
