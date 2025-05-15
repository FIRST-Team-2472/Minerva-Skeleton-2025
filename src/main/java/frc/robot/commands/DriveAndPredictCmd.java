package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AimPoint;
import frc.robot.AutoAiming;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.ArmSubsystems.PitchMotorSubsystem;
import frc.robot.subsystems.ArmSubsystems.ShootingMotorSubsystem;
import frc.robot.subsystems.swerveExtras.FieldPose2d;
import frc.robot.subsystems.swerveExtras.PosPose2d;

public class DriveAndPredictCmd extends Command {

    SwerveSubsystem swerveSubsystem;
    PitchMotorSubsystem pitchMotorSubsystem;
    ShootingMotorSubsystem shootingMotorSubsystem;
    FieldPose2d targetPosition;
    AimPoint aimPoint;
    Timer timer, overrideTimer;

    /**
     * moves the robot to a target position and prepares to shoot by angling the
     * arm, choosing the robot angle and revving the shooter motor
     * 
     * @param swerveSubsystem        the swerve subsytstem
     * @param pitchMotorSubsystem    the pitch motor subsystem
     * @param shootingMotorSubsystem the shooting motor subsystem
     * @param targetPosition         the position to move to and prepare to shoot
     * 
     * 
     */
    public DriveAndPredictCmd(SwerveSubsystem swerveSubsystem, PitchMotorSubsystem pitchMotorSubsystem,
            ShootingMotorSubsystem shootingMotorSubsystem, PosPose2d targetPosition) {
        this.swerveSubsystem = swerveSubsystem;
        this.pitchMotorSubsystem = pitchMotorSubsystem;
        this.shootingMotorSubsystem = shootingMotorSubsystem;
        aimPoint = AutoAiming.getAimPoint(targetPosition);
        this.targetPosition = new FieldPose2d(targetPosition.toFieldPose2d().getX(),
                targetPosition.toFieldPose2d().getY(), Rotation2d.fromDegrees(aimPoint.getYawAngle()));

        timer = new Timer();

        addRequirements(swerveSubsystem);
    }

    public void initialize() {
        swerveSubsystem.initializeDriveToPointAndRotate();
        timer.restart();
        overrideTimer.restart();
    }

    @Override
    public void execute() {
        swerveSubsystem.executeDriveToPointAndRotate(targetPosition);
        pitchMotorSubsystem.runPitchMotorWithKP(aimPoint.getPitchAngle());
        if (AutoAiming.isWithinDistance(targetPosition, swerveSubsystem.getPose(), 1))
            shootingMotorSubsystem.runShooterMotorsWithKP(3500);
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        // use this function if you overide the command to finsih it
        if (swerveSubsystem.isAtPoint(targetPosition.getTranslation())
                && swerveSubsystem.isAtAngle(targetPosition.getRotation())
                && (Math.abs(pitchMotorSubsystem.getEncoderDeg() - aimPoint.getPitchAngle()) < 0.5) || (overrideTimer.hasElapsed(3)))
            return timer.hasElapsed(.05);
        timer.restart();
        return false;
    }
}
