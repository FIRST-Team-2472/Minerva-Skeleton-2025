package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.swerveExtras.PosPose2d;
import frc.robot.subsystems.swerveExtras.PositivePoint;

import java.util.ArrayList;
import java.util.List;

public class CommandSequences {

    PosPose2d[] miscellaneousNodes = new PosPose2d[4];
    PosPose2d[] importantNodes = new PosPose2d[6];
    PosPose2d[] startingNodes = new PosPose2d[5];
    PosPose2d[] collectingNearNodes = new PosPose2d[3];
    PosPose2d[] shootingNearNodes = new PosPose2d[3];
    PosPose2d ampNode = simplePose(1.84, 7.32, -90);

    public CommandSequences() {

        // by the source over the line
        miscellaneousNodes[0] = simplePose(3, 2, 0);
        // On top of note 2
        miscellaneousNodes[1] = simplePose(2.91, 5.4, 0);
        // On the way to note 1
        miscellaneousNodes[2] = simplePose(1.76, 7, 0);
        miscellaneousNodes[3] = simplePose(1, 1, 0);

        // non-amp side of Speaker
        importantNodes[0] = simplePose(.55, 4.10, 0);
        // In front of Note
        importantNodes[1] = simplePose(2.2, 4.10, 0);
        // Near front of Speaker
        importantNodes[2] = simplePose(2.2, 5.57, 0);
        // In from of amp
        importantNodes[3] = simplePose(1.84, 7.32, -130);
        // amp side of stage
        importantNodes[4] = simplePose(4.28, 6.30, 0);
        // under the stage
        importantNodes[5] = simplePose(4.78, 4.15, 0);

        // amp start
        startingNodes[0] = simplePose(1.41, 7.26, 0);
        // speaker start 1
        startingNodes[1] = simplePose(0.71, 6.7, 60);
        // speaker start 2
        startingNodes[2] = simplePose(1.4, 5.52, 0);
        // speaker start 3
        startingNodes[3] = simplePose(0.71, 4.38, -60);

        startingNodes[4] = simplePose(2.46, 7.27, 0);

        // Collecting the near nodes
        collectingNearNodes[0] = simplePose(2.5, 6.92, 0);
        collectingNearNodes[1] = simplePose(2.15, 5.5, 0); // same as imp. n. [2];
        collectingNearNodes[2] = simplePose(2.4, 3.93, 0); // same as imp. n. [3];

        // Shooting to the speaker from the near nodes
        shootingNearNodes[0] = simplePose(2.9, 7, 30);
        shootingNearNodes[1] = simplePose(2.9, 5.5, 0);
        shootingNearNodes[2] = simplePose(2.9, 4.08, -30);
    }

    // generates a path via points
    private static Command generatePath(SwerveSubsystem swerveSubsystem, PosPose2d startPoint,
            List<PositivePoint> midPoints,
            PosPose2d endPoint) {
        // 1. Create trajectory settings
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(DriveConstants.kDriveKinematics);

        Pose2d driveStartPoint = startPoint.toFieldPose2d();
        Pose2d driveEndPoint = endPoint.toFieldPose2d();
        List<Translation2d> driveMidPoints = new ArrayList<Translation2d>();
        for (int i = 0; i < midPoints.size(); i++)
            driveMidPoints.add(midPoints.get(i).toFieldPos());

        // 2. Generate trajectory
        // Generates trajectory. Need to feed start point, a series of inbetween points,
        // and end point
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                driveStartPoint,
                driveMidPoints,
                driveEndPoint,
                trajectoryConfig);

        // 3. Define PID controllers for tracking trajectory
        SwerveControllerCommand swerveControllerCommand = getSwerveControllerCommand(swerveSubsystem, trajectory);

        // 5. Add some init and wrap-up, and return everything
        // creates a Command list that will reset the Odometry, then move the path, then
        // stop
        return new SequentialCommandGroup(
                swerveControllerCommand,
                new InstantCommand(() -> swerveSubsystem.stopModules()));
    }

    private static SwerveControllerCommand getSwerveControllerCommand(SwerveSubsystem swerveSubsystem, Trajectory trajectory) {
        PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
        PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(
                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // 4. Construct command to follow trajectory
        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                trajectory,
                // swerveSubsystm::getPose is same as () -> swerveSubsystem.getPose()
                swerveSubsystem::getPose,
                DriveConstants.kDriveKinematics,
                xController,
                yController,
                thetaController,
                swerveSubsystem::setModuleStates,
                swerveSubsystem);
        return swerveControllerCommand;
    }

    public PosPose2d simplePose(double x, double y, double angleDegrees) {
        return new PosPose2d(x, y, Rotation2d.fromDegrees(angleDegrees));
    }
    public static Rotation2d teamChangeAngle(double degrees){
        if(SwerveSubsystem.isOnRed())
                return  Rotation2d.fromDegrees(-degrees+180);
        return  Rotation2d.fromDegrees(degrees);
    }

}