package frc.robot;

import java.util.ArrayList;
import java.util.List;

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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ArmMotorsConstants;
import frc.robot.Constants.AutoAimingConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ArmMotorsConstants.PitchMotor;
import frc.robot.subsystems.*;
import frc.robot.subsystems.ArmSubsystems.*;
import frc.robot.subsystems.swerveExtras.*;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

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

    public Command test(SwerveSubsystem swerveSubsystem, PitchMotorSubsystem pitchMotorSubsystem, ShootingMotorSubsystem shootingMotorSubsystem, IntakeMotorSubsystem intakeMotorSubsystem) {

        swerveSubsystem.resetOdometry(simplePose(2, 5.55, 0).toFieldPose2d());

        return new SequentialCommandGroup(
                new SetArmPitchCmd(pitchMotorSubsystem, ArmMotorsConstants.PitchMotor.kPitchMotorSpeakerPresetAngle),
                new IntakeNoteCmd(intakeMotorSubsystem, pitchMotorSubsystem, 0, 5)
        );
    }

    public Command driveFromZone(SwerveSubsystem swerveSubsystem) {

        System.out.println("Autos Happening");
        System.out.println(miscellaneousNodes[0].toString());
        swerveSubsystem.resetOdometry(startingNodes[0]);

        return new SequentialCommandGroup(
                generatePath(swerveSubsystem, startingNodes[0], List.of(), startingNodes[4]));
    }

    public Command twoInSpeakerFromPositionTwoCommand(SwerveSubsystem swerveSubsystem,
            PitchMotorSubsystem pitchMotorSubsystem, ShootingMotorSubsystem shooterSubsystem,
            IntakeMotorSubsystem intakeMotorSubsystem) {

        System.out.println("Autos Happening");
        System.out.println(miscellaneousNodes[0].toString());

        swerveSubsystem.resetOdometry(startingNodes[2].toFieldPose2d());

        return new SequentialCommandGroup(
                new SetArmPitchCmd(pitchMotorSubsystem, ArmMotorsConstants.PitchMotor.kPitchMotorSpeakerPresetAngle),
                new ShootNoteCmd(shooterSubsystem, intakeMotorSubsystem, 0.7),
                new ParallelCommandGroup(
                        new IntakeNoteCmd(intakeMotorSubsystem, pitchMotorSubsystem, 0, 3.2),
                        new SwerveDriveToPointCmd(swerveSubsystem, miscellaneousNodes[1])
                ),
                new FastAutoAimCmd(pitchMotorSubsystem, swerveSubsystem, shooterSubsystem, intakeMotorSubsystem)
        ); 
                /* new SetArmPitchCmd(pitchMotorSubsystem, ArmMotorsConstants.PitchMotor.kPitchMotorSpeakerPresetAngle+.5),
                new runShooter(shooterSubsystem, intakeMotorSubsystem, 0.7),
                                new SequentialCommandGroup(
                                        new SetArmPitchCmd(pitchMotorSubsystem,
                                ArmMotorsConstants.PitchMotor.kPitchMotorIntakePresetAngle),
                        new SwerveDriveToPointCmd(swerveSubsystem, miscellaneousNodes[1]),
                        new IntakeNoteCmd(intakeMotorSubsystem, 0, 2.3)),
                new SetArmPitchCmd(pitchMotorSubsystem, ArmMotorsConstants.PitchMotor.kPitchMotorFarSpeakerPresetAngle),
                new ShootNoteCmd(shooterSubsystem, intakeMotorSubsystem, 0.7));
    }

    public Command boo(SwerveSubsystem swerveSubsystem, PitchMotorSubsystem pitchMotorSubsystem,
            ShootingMotorSubsystem shooterSubsystem, IntakeMotorSubsystem intakeMotorSubsystem) {

        System.out.println("An auto is happening");
        swerveSubsystem.resetOdometry(startingNodes[1]);

        return new ParallelCommandGroup(
                new SwerveDriveToPointCmd(swerveSubsystem, importantNodes[3]),
                new SetArmPitchCmd(pitchMotorSubsystem, PitchMotor.kPitchMotorAmpPresetAngle),
                new ShootNoteCmd(shooterSubsystem, intakeMotorSubsystem, .4));
    }

    public Command twoInSpeakerFromPositionOneCommand(SwerveSubsystem swerveSubsystem,
            PitchMotorSubsystem pitchMotorSubsystem, ShootingMotorSubsystem shooterSubsystem,
            IntakeMotorSubsystem intakeMotorSubsystem) {
        swerveSubsystem.resetOdometry(startingNodes[1].toFieldPose2d());

        return new SequentialCommandGroup(
                new SetArmPitchCmd(pitchMotorSubsystem, 77),
                new ShootNoteCmd(shooterSubsystem, intakeMotorSubsystem, 0.7),
                new ParallelCommandGroup(
                        new SetArmPitchCmd(pitchMotorSubsystem,
                                ArmMotorsConstants.PitchMotor.kPitchMotorIntakePresetAngle),
                        new SwerveDriveToPointCmd(swerveSubsystem, collectingNearNodes[0]),
                        new IntakeNoteCmd(intakeMotorSubsystem, .5, 3.2)
                ),
                RotateNShoot(swerveSubsystem, pitchMotorSubsystem, shooterSubsystem, intakeMotorSubsystem));
    }

    public Command fourNoteFromPosTwo(SwerveSubsystem swerveSubsystem, PitchMotorSubsystem pitchMotorSubsystem,
            ShootingMotorSubsystem shooterSubsystem, IntakeMotorSubsystem intakeMotorSubsystem) {
                
        swerveSubsystem.resetOdometry(startingNodes[2].toFieldPose2d());

        return new SequentialCommandGroup(
                // should be changed to autoshooting once you fix the schedule issue
                new FastAutoAimCmd(pitchMotorSubsystem, swerveSubsystem, shooterSubsystem, intakeMotorSubsystem),
                new ParallelCommandGroup(
                        new SwerveDriveToPointCmd(swerveSubsystem, simplePose(2.52, 4.08, 0)),
                        new SetArmPitchCmd(pitchMotorSubsystem, ArmMotorsConstants.PitchMotor.kPitchMotorIntakePresetAngle),
                        new IntakeNoteCmd(intakeMotorSubsystem, 0, 3)
                ),
                new FastAutoAimCmd(pitchMotorSubsystem, swerveSubsystem, shooterSubsystem, intakeMotorSubsystem),
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new SwerveRotateToAngle(swerveSubsystem, Rotation2d.fromDegrees(-135)),
                                new ParallelCommandGroup(
                                        new SwerveDriveToPointCmd(swerveSubsystem, simplePose(3.1, 5.5, -90)),
                                        new IntakeNoteCmd(intakeMotorSubsystem, 0, 3)
                                )
                        ),
                        new SetArmPitchCmd(pitchMotorSubsystem,
                                ArmMotorsConstants.PitchMotor.kPitchMotorIntakePresetAngle)
                ),
                new FastAutoAimCmd(pitchMotorSubsystem, swerveSubsystem, shooterSubsystem, intakeMotorSubsystem),
                new ParallelCommandGroup(
                        new SwerveDriveToPointCmd(swerveSubsystem, simplePose(2.71, 6.9, -90)),
                        new SetArmPitchCmd(pitchMotorSubsystem,
                                ArmMotorsConstants.PitchMotor.kPitchMotorIntakePresetAngle),
                        new IntakeNoteCmd(intakeMotorSubsystem, 0.2, 3)),
                new FastAutoAimCmd(pitchMotorSubsystem, swerveSubsystem, shooterSubsystem, intakeMotorSubsystem));
                /* new FastAutoAimCmd(pitchMotorSubsystem, swerveSubsystem, shooterSubsystem, intakeMotorSubsystem),
                new ParallelCommandGroup(
                        new SwerveDriveToPointCmd(swerveSubsystem, simplePose(2.71, 6.9, 40)),
                        new SetArmPitchCmd(pitchMotorSubsystem,
                                ArmMotorsConstants.PitchMotor.kPitchMotorIntakePresetAngle),
                        new IntakeNoteCmd(intakeMotorSubsystem, 0.2, 3)),
                new FastAutoAimCmd(pitchMotorSubsystem, swerveSubsystem, shooterSubsystem, intakeMotorSubsystem),

                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new SwerveRotateToAngle(swerveSubsystem, Rotation2d.fromDegrees(-135)),
                                new ParallelCommandGroup(
                                        new SwerveDriveToPointCmd(swerveSubsystem, simplePose(3.1, 5.5, -90)),
                                        new IntakeNoteCmd(intakeMotorSubsystem, 0, 1.5)
                                )
                        ),
                        new SetArmPitchCmd(pitchMotorSubsystem,
                                ArmMotorsConstants.PitchMotor.kPitchMotorIntakePresetAngle)
                ),
                new FastAutoAimCmd(pitchMotorSubsystem, swerveSubsystem, shooterSubsystem, intakeMotorSubsystem),
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new SwerveRotateToAngle(swerveSubsystem,  Rotation2d.fromDegrees(-90)),
                        new SwerveDriveToPointCmd(swerveSubsystem, simplePose(2.91, 4.08, -90))
                        ),
                        new SetArmPitchCmd(pitchMotorSubsystem,
                                ArmMotorsConstants.PitchMotor.kPitchMotorIntakePresetAngle),
                        new IntakeNoteCmd(intakeMotorSubsystem, 1.5, 4)),
                new FastAutoAimCmd(pitchMotorSubsystem, swerveSubsystem, shooterSubsystem, intakeMotorSubsystem)); */
    }

    public Command shootWaitNCrossTheLineAmpSide(SwerveSubsystem swerveSubsystem, PitchMotorSubsystem pitchMotorSubsystem,
            ShootingMotorSubsystem shooterSubsystem, IntakeMotorSubsystem intakeMotorSubsystem){
                
                swerveSubsystem.resetOdometry(startingNodes[1].toFieldPose2d());

                return new SequentialCommandGroup(
                        new SetArmPitchCmd(pitchMotorSubsystem, ArmMotorsConstants.PitchMotor.kPitchMotorSpeakerPresetAngle),
                        new ShootNoteCmd(shooterSubsystem, intakeMotorSubsystem, 0.9),
                        new WaitCommand(9),
                        new SwerveDriveToPointCmd(swerveSubsystem, simplePose(3.2, 7, 0))
                );
            }

    public Command threeNoteFromPosTwo(SwerveSubsystem swerveSubsystem, PitchMotorSubsystem pitchMotorSubsystem,
            ShootingMotorSubsystem shooterSubsystem, IntakeMotorSubsystem intakeMotorSubsystem) {
                
        swerveSubsystem.resetOdometry(startingNodes[2].toFieldPose2d());

        return new SequentialCommandGroup(
                // should be changed to autoshooting once you fix the schedule issue
                new FastAutoAimCmd(pitchMotorSubsystem, swerveSubsystem, shooterSubsystem, intakeMotorSubsystem),
                new ParallelCommandGroup(
                        new SwerveDriveToPointCmd(swerveSubsystem, simplePose(2.8, 6.92, 40)),
                        new SetArmPitchCmd(pitchMotorSubsystem,
                                ArmMotorsConstants.PitchMotor.kPitchMotorIntakePresetAngle),
                        new IntakeNoteCmd(intakeMotorSubsystem, 0.2, 3.5)),
                new FastAutoAimCmd(pitchMotorSubsystem, swerveSubsystem, shooterSubsystem, intakeMotorSubsystem),

                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                        new SwerveRotateToAngle(swerveSubsystem, Rotation2d.fromDegrees(-135)),
                                        new SetArmPitchCmd(pitchMotorSubsystem, ArmMotorsConstants.PitchMotor.kPitchMotorIntakePresetAngle)
                                ),
                                new ParallelCommandGroup(
                                        new SwerveDriveToPointCmd(swerveSubsystem, simplePose(3.1, 5.5, -90)),
                                        new IntakeNoteCmd(intakeMotorSubsystem, 0, 2)
                                )
                        )
                ),
                new FastAutoAimCmd(pitchMotorSubsystem, swerveSubsystem, shooterSubsystem, intakeMotorSubsystem)
        );
    }

    public Command twoInSpeakerFromPositionThreeCommand(SwerveSubsystem swerveSubsystem,
            PitchMotorSubsystem pitchMotorSubsystem, ShootingMotorSubsystem shooterSubsystem,
            IntakeMotorSubsystem intakeMotorSubsystem) {
        swerveSubsystem.resetOdometry(startingNodes[3].toFieldPose2d());

        return new SequentialCommandGroup(
                new SetArmPitchCmd(pitchMotorSubsystem,77),
                new ShootNoteCmd(shooterSubsystem, intakeMotorSubsystem, 0.7),
                new SetArmPitchCmd(pitchMotorSubsystem, ArmMotorsConstants.PitchMotor.kPitchMotorIntakePresetAngle),
                new ParallelCommandGroup(
                        new SetArmPitchCmd(pitchMotorSubsystem,
                                ArmMotorsConstants.PitchMotor.kPitchMotorIntakePresetAngle),
                        new SwerveDriveToPointCmd(swerveSubsystem, collectingNearNodes[2]),
                        new IntakeNoteCmd(intakeMotorSubsystem, .5, 3.2)
                ),
                RotateNShoot(swerveSubsystem, pitchMotorSubsystem, shooterSubsystem, intakeMotorSubsystem));
    }

    public Command oneInAmpOneFromSpeakerPositionOneCommand(SwerveSubsystem swerveSubsystem) {
        swerveSubsystem.resetOdometry(startingNodes[0]);

        return new SequentialCommandGroup(
                generatePath(swerveSubsystem, startingNodes[1], List.of(), collectingNearNodes[0]),
                generatePath(swerveSubsystem, collectingNearNodes[0], List.of(), importantNodes[3]));
    }

    public Command threeInSpeakerFromPositionOneCommand(SwerveSubsystem swerveSubsystem) {
        swerveSubsystem.resetOdometry(startingNodes[0]);

        return new SequentialCommandGroup(
                generatePath(swerveSubsystem, startingNodes[1], List.of(), collectingNearNodes[0]),
                generatePath(swerveSubsystem, collectingNearNodes[0], List.of(), importantNodes[3]));
    }

    public Command underStage(SwerveSubsystem swerveSubsystem) {

        System.out.println("Autos Happening");
        System.out.println(miscellaneousNodes[0].toString());
        swerveSubsystem.resetOdometry(startingNodes[0]);

        return new SequentialCommandGroup(
                generatePath(swerveSubsystem, startingNodes[0], List.of(importantNodes[4].getPositivePoint()),
                        importantNodes[5]));
    }

    public Command justShoot(SwerveSubsystem swerveSubsystem, PitchMotorSubsystem pitchMotorSubsystem, ShootingMotorSubsystem shooterSubsystem,
            IntakeMotorSubsystem intakeMotorSubsystem) {

        return new SequentialCommandGroup(
                new SetArmPitchCmd(pitchMotorSubsystem, ArmMotorsConstants.PitchMotor.kPitchMotorSpeakerPresetAngle),
                new ShootNoteCmd(shooterSubsystem, intakeMotorSubsystem, 0.9)
        );
    }

    public Command justShootAndMove(SwerveSubsystem swerveSubsystem, PitchMotorSubsystem pitchMotorSubsystem,
            ShootingMotorSubsystem shooterSubsystem, IntakeMotorSubsystem intakeMotorSubsystem) {

        swerveSubsystem.resetOdometry(startingNodes[3].toFieldPose2d());

        return new SequentialCommandGroup(
                new SetArmPitchCmd(pitchMotorSubsystem, ArmMotorsConstants.PitchMotor.kPitchMotorSpeakerPresetAngle),
                new ShootNoteCmd(shooterSubsystem, intakeMotorSubsystem, 0.7),
                generatePath(swerveSubsystem, startingNodes[3], List.of(), miscellaneousNodes[0]));
    }

    public Command justMovePositionTwoToNoteTwoCommand(SwerveSubsystem swerveSubsystem) {

        swerveSubsystem.resetOdometry(startingNodes[2].toFieldPose2d());

        return new SequentialCommandGroup(
                generatePath(swerveSubsystem, startingNodes[2], List.of(), importantNodes[2]));
    }

    public Command justRunIntake(IntakeMotorSubsystem intakeMotorSubsystem) {

        return new SequentialCommandGroup(
                new IntakeNoteCmd(intakeMotorSubsystem, 0, 0.6));
    }

    public Command shootFromFurtherAway(PitchMotorSubsystem pitchMotorSubsystem,
            ShootingMotorSubsystem shooterSubsystem, IntakeMotorSubsystem intakeMotorSubsystem) {

        return new SequentialCommandGroup(
                new SetArmPitchCmd(pitchMotorSubsystem, ArmMotorsConstants.PitchMotor.kPitchMotorFarSpeakerPresetAngle),
                new ShootNoteCmd(shooterSubsystem, intakeMotorSubsystem, 0.7));
    }

    public Command justMovePositionTwoToNoteOne(SwerveSubsystem swerveSubsystem) {

        swerveSubsystem.resetOdometry(startingNodes[2].toFieldPose2d());

        return new SequentialCommandGroup(
          new SwerveDriveToPointCmd(swerveSubsystem, shootingNearNodes[0])      
        );
    }

    public Command justMovePositionTwoToNoteThreeCommand(SwerveSubsystem swerveSubsystem) {

        swerveSubsystem.resetOdometry(startingNodes[2].toFieldPose2d());

        return new SequentialCommandGroup(
                new SwerveDriveToPointCmd(swerveSubsystem, importantNodes[1]));
    }

    public Command justMovePositionOneToNoteOneCommand(SwerveSubsystem swerveSubsystem) {

        swerveSubsystem.resetOdometry(startingNodes[1].toFieldPose2d());

        return new SequentialCommandGroup(
                new SwerveDriveToPointCmd(swerveSubsystem, collectingNearNodes[0]));
    }

    public Command justMovePositionOneToNoteTwoCommand(SwerveSubsystem swerveSubsystem) {

        swerveSubsystem.resetOdometry(startingNodes[1].toFieldPose2d());

        return new SequentialCommandGroup(
                new SwerveDriveToPointCmd(swerveSubsystem, importantNodes[2]));
    }

    public Command justMovePosition1tonote3(SwerveSubsystem swerveSubsystem) {

        swerveSubsystem.resetOdometry(startingNodes[1].toFieldPose2d());

        return new SequentialCommandGroup(
                new SwerveDriveToPointCmd(swerveSubsystem, importantNodes[1]));
    }
    public Command fiveNoteFromPosition2(SwerveSubsystem swerveSubsystem, PitchMotorSubsystem pitchMotorSubsystem, ShootingMotorSubsystem shooterSubsystem, IntakeMotorSubsystem intakeMotorSubsystem){

        swerveSubsystem.resetOdometry(startingNodes[2].toFieldPose2d());

        return new SequentialCommandGroup(
            RotateNShoot(swerveSubsystem, pitchMotorSubsystem, shooterSubsystem, intakeMotorSubsystem),

            new ParallelCommandGroup(
                new SetArmPitchCmd(pitchMotorSubsystem, ArmMotorsConstants.PitchMotor.kPitchMotorIntakePresetAngle),
                new IntakeNoteCmd(intakeMotorSubsystem, 0.2, 3),//may change
                new SwerveDriveToPointCmd(swerveSubsystem, simplePose(2.3, 5.52, 0))
            ),
            RotateNShoot(swerveSubsystem, pitchMotorSubsystem, shooterSubsystem, intakeMotorSubsystem),

            new ParallelCommandGroup(
                new SwerveRotateToAngle(swerveSubsystem, Rotation2d.fromDegrees(66)),
                new SetArmPitchCmd(pitchMotorSubsystem, ArmMotorsConstants.PitchMotor.kPitchMotorIntakePresetAngle)
            ),
            new ParallelCommandGroup(
                new SwerveDriveToPointCmd(swerveSubsystem, simplePose(2.65, 6.45, 66)),
                new IntakeNoteCmd(intakeMotorSubsystem, 0.2, 3)//may change)
            ),
            RotateNShoot(swerveSubsystem, pitchMotorSubsystem, shooterSubsystem, intakeMotorSubsystem),

            new ParallelCommandGroup(
                new SetArmPitchCmd(pitchMotorSubsystem, ArmMotorsConstants.PitchMotor.kPitchMotorIntakePresetAngle),
                new IntakeNoteCmd(intakeMotorSubsystem, 0.2, 3),//may change
                new SwerveDriveToPointCmd(swerveSubsystem, simplePose(7.77, 7.35, 9.95))
            ),
            new SwerveDriveToPointCmd(swerveSubsystem, simplePose(5.85, 6.9, 16)),
            RotateNShoot(swerveSubsystem, pitchMotorSubsystem, shooterSubsystem, intakeMotorSubsystem),
            
            new ParallelCommandGroup(
                new SwerveRotateToAngle(swerveSubsystem, Rotation2d.fromDegrees(-24.5)),
                new SetArmPitchCmd(pitchMotorSubsystem, ArmMotorsConstants.PitchMotor.kPitchMotorIntakePresetAngle)
            ),
            new ParallelCommandGroup(
                new SwerveDriveToPointCmd(swerveSubsystem, simplePose(7.75, 6.05, -24.5)),
                new IntakeNoteCmd(intakeMotorSubsystem, 0.2, 3)//may change)
            ),
            new SwerveDriveToPointCmd(swerveSubsystem, simplePose(5.86, 6.4, 9)),
            RotateNShoot(swerveSubsystem, pitchMotorSubsystem, shooterSubsystem, intakeMotorSubsystem)
        );
    }

    public Command RotateNShoot(SwerveSubsystem swerveSubsystem,
            PitchMotorSubsystem pitchMotorSubsystem, ShootingMotorSubsystem shootingMotorSubsystem,
            IntakeMotorSubsystem intakeMotorSubsystem) {

        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new SetArmPitchCmd(pitchMotorSubsystem,  swerveSubsystem),
                        new SwerveRotateToAngle(swerveSubsystem)),
                new ShootNoteCmd(shootingMotorSubsystem, intakeMotorSubsystem, 0.9, 4000));
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

        // 5. Add some init and wrap-up, and return everything
        // creates a Command list that will reset the Odometry, then move the path, then
        // stop
        return new SequentialCommandGroup(
                swerveControllerCommand,
                new InstantCommand(() -> swerveSubsystem.stopModules()));
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