package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmMotorsConstants;
import frc.robot.subsystems.ArmSubsystems.*;

public class IntakeNoteCmd extends Command {

    private IntakeMotorSubsystem intakeSubsystem;
    private PitchMotorSubsystem pitchMotorSubsystem;
    private Timer timer;
    private double startingDelay, endingDelay;

    public IntakeNoteCmd(IntakeMotorSubsystem intakeSubsystem, double startingDelay, double endingDelay) {
        timer = new Timer();
        this.intakeSubsystem = intakeSubsystem;
        this.startingDelay = startingDelay;
        this.endingDelay = endingDelay;
        addRequirements(intakeSubsystem);
    }

    public IntakeNoteCmd(IntakeMotorSubsystem intakeSubsystem, PitchMotorSubsystem pitchMotorSubsystem,
            double startingDelay, double endingDelay) {
        this(intakeSubsystem, startingDelay, endingDelay);
        this.pitchMotorSubsystem = pitchMotorSubsystem;
        addRequirements(pitchMotorSubsystem);
    }

    @Override
    public void initialize() {
        timer.restart();
        System.out.println("intake command started");
    }

    @Override
    public void execute() {
        if (pitchMotorSubsystem != null) {
            if (Math.abs(pitchMotorSubsystem.getEncoderDeg()
                    - ArmMotorsConstants.PitchMotor.kPitchMotorIntakePresetAngle) > 5) {
                pitchMotorSubsystem
                        .runPitchMotorWithFasterKP(ArmMotorsConstants.PitchMotor.kPitchMotorIntakePresetAngle);
            } else {
                pitchMotorSubsystem.runPitchMotor(0, true);
            }
        }
        if (timer.hasElapsed(startingDelay)) {
            System.out.println("should be running motors");
            intakeSubsystem.runIntakeMotors(1);
            intakeSubsystem.runPushMotor(16);
        }
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.runIntakeMotors(0);
        intakeSubsystem.runPushMotor(0);
        System.out.println("intake command ended, time: " + timer.get());
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(endingDelay) || intakeSubsystem.getPhotoElectricSensor() == true;
    }
}
