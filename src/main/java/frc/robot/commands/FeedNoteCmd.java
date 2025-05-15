package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystems.IntakeMotorSubsystem;

public class FeedNoteCmd extends Command {
    IntakeMotorSubsystem intakeSubsystem;
    Timer timer = new Timer();

    public FeedNoteCmd(IntakeMotorSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;

        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        timer.reset();
        intakeSubsystem.runIntakeMotors(1.0);
        intakeSubsystem.runPushMotor(1.0);
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.runIntakeMotors(0);
        intakeSubsystem.runPushMotor(0);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(0.5);
    }
}
