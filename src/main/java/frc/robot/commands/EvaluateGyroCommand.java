package frc.robot.commands;

import com.ctre.phoenix6.spns.SpnValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.SwerveSubsystem;

public class EvaluateGyroCommand extends Command {
    private SwerveSubsystem swerveSubsystem;
    private String limeLightName = "limelight-idk";

    private int rotations;                  // How many rotations to perform per test
    private double rotateBy;                // How many degrees we rotate by with each test
    private double targetAngle;             // Where we want to be at the end of a spin, updates with each test
    private double angleDifference;         // How many degrees we have left to go

    private boolean isDone = false;         // Are we done yet?
    private Stage stage = Stage.STARTING;   // Contains the current state of the state machine

    private int testsDone = 0;              // How many tests we have already completed
    private int totalTests = 0;             // How many tests we need total

    private PIDController rotationalPID = new PIDController(0.2, 0.0, 0.02);
    private double maxRotationalVelocity = -Constants.TargetPosConstants.kMaxAngularSpeed;

    private double angularTolerance = 5;    // How many degrees off we call good enough
                                            // Should not affect measurement too much
                                            // as the Pigeon2 should still know if it's off
    private double velocityTolerance = 0.01; // How slow should we be going before allowing us to stop

    private double initialCameraAngle = 0.0; // The angle the camera sees at the beginning of a test
    private double finalCameraAngle = 0.0;  // The angle the camera sees at the end of a test
    private double finalGyroAngle = 0.0;  // The

    public EvaluateGyroCommand(SwerveSubsystem swerveSubsystem, int rotations, int tests) {
        this.rotations = rotations;
        this.rotateBy = rotations * 360;
        this.swerveSubsystem = swerveSubsystem;
        this.totalTests = tests;

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        this.stage = Stage.STARTING;
    }

    @Override
    public void execute() {

        if (this.testsDone < this.totalTests) {
            switch (this.stage) {

                case STARTING:
                    this.initialCameraAngle = getCurrentCameraAngle();
                    if ((testsDone % 2) == 0) {
                        this.targetAngle = getCurrentGyroAngle() + rotateBy; // One way
                    } else {
                        this.targetAngle = getCurrentGyroAngle() - rotateBy; // then the other!
                    }

                    this.stage = Stage.SPINNING;
                    break;

                case SPINNING:
                    double turningSpeed = this.getTurningSpeed();
                    this.swerveSubsystem.runModulesFieldRelative(0, 0, turningSpeed);

                    if (this.isCloseEnough(this.angleDifference, turningSpeed)) {
                        this.swerveSubsystem.stopModules();
                        this.finalCameraAngle = this.getCurrentCameraAngle();
                        this.finalGyroAngle = this.getCurrentGyroAngle();
                        this.stage = Stage.CALCULATING;
                    }
                    break;

                case CALCULATING:
                    System.out.println("+-------------- Pigeon2 Report --------------+");

                    double degreesError = this.finalCameraAngle - this.initialCameraAngle;
                    System.out.println("Error reported by the camera is: " + degreesError + "°");
                    System.out.println(); // TODO: Ensure the gyro and camera move the same direction

                    double knownError = this.targetAngle - this.finalGyroAngle;
                    System.out.println("But the gyro knows its off by:   " + knownError + "°");
                    System.out.println("This error can be assumed to be coming from the PID!");
                    System.out.println();

                    degreesError -= knownError; // TODO make sure that this is `-`
                    System.out.println("So the real error is:            " + degreesError + "°");
                    System.out.println();

                    double errorPerRotation = degreesError / this.rotations;
                    System.out.println("The error per rotation:          " + errorPerRotation + "°");
                    System.out.println();

                    double errorPerDegree = errorPerRotation / 360.0;
                    System.out.println("The error per degree:            " + errorPerDegree + "°");
                    System.out.println();

                    double angleRatio = (360.0 + errorPerRotation) / 360.0;
                    System.out.println("Each gyro rotation is actually:  " + angleRatio + " rots");
                    System.out.println();

                    double correctionRatio = 1.0 / angleRatio;
                    System.out.println("Multiply the zeroed heading by:  " + correctionRatio);
                    System.out.println("to correct this error");
                    System.out.println("(if the error is constant and linear)");
                    System.out.println();

                    System.out.println("+--------------------------------------------+");

                    this.stage = Stage.DONE;

                case DONE:
                    this.testsDone++;
            }
            return;
        }
        this.isDone = true;
    }

    private double getCurrentCameraAngle() {
        Pose3d targetPose = LimelightHelpers.getTargetPose3d_RobotSpace(this.limeLightName);
        double angleRadians = Math.atan2(targetPose.getX(), targetPose.getY());

        return Math.toDegrees(angleRadians);
    }

    private double getCurrentGyroAngle() {
        return this.swerveSubsystem.getPigeonRaw(SpnValue.Pigeon2AccumGyroZ.value);
    }

    private boolean isCloseEnough(double angle, double velocity) {
        return (Math.abs(velocity) <= this.velocityTolerance) | (Math.abs(angle) <= this.angularTolerance);
    }

    private double getTurningSpeed() {
        this.angleDifference = this.targetAngle - this.getCurrentGyroAngle();
        // System.out.println("Angle Error: " + angleDifference);
        return MathUtil.clamp(this.rotationalPID.calculate(Math.toRadians(this.angleDifference),
                0), -1, 1) * this.maxRotationalVelocity;
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return this.isDone;
    }
}

enum Stage {
    STARTING,       // Start of *a* test
    SPINNING,       // Weeee!
    CALCULATING,    // Calculate error metrics
    CORRECTING,     // In case the error is great enough that we have to re-adjust each test
    DONE            // Done with *a* test, not all
}
