package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.DeviceIdentifier;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.jni.StatusSignalJNI;
import com.ctre.phoenix6.spns.SpnValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SensorConstants;
import frc.robot.Constants.TargetPosConstants;
import frc.robot.subsystems.swerveExtras.AccelerationLimiter;
import frc.robot.subsystems.swerveExtras.SwerveModule;
import org.littletonrobotics.junction.Logger;

import java.util.Optional;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveModule frontLeft = new SwerveModule(//
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveEncoderReversed,
            DriveConstants.kFrontLeftTurningEncoderReversed,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetAng,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule frontRight = new SwerveModule(//
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveEncoderReversed,
            DriveConstants.kFrontRightTurningEncoderReversed,
            DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
            DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetAng,
            DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

    private final SwerveModule backLeft = new SwerveModule(//
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveEncoderReversed,
            DriveConstants.kBackLeftTurningEncoderReversed,
            DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
            DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetAng,
            DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule backRight = new SwerveModule(//
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveEncoderReversed,
            DriveConstants.kBackRightTurningEncoderReversed,
            DriveConstants.kBackRightDriveAbsoluteEncoderPort,
            DriveConstants.kBackRightDriveAbsoluteEncoderOffsetAng,
            DriveConstants.kBackRightDriveAbsoluteEncoderReversed);

    private final AccelerationLimiter xLimiter, yLimiter, turningLimiter;
    private PIDController xController, yController;

    public PIDController thetaController;

    private final Pigeon2 gyro = new Pigeon2(SensorConstants.kPigeonID);
    public final SwerveDriveOdometry odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics,
            new Rotation2d(0), getModulePositions());

    private static final SendableChooser<String> colorChooser = new SendableChooser<>();
    private final String red = "Red", blue = "Blue";
    private Pose2d lastSeenPosition;

    private boolean camsDisabled, constantAim;

    ChassisSpeeds chassisSpeeds = new ChassisSpeeds();

    public SwerveSubsystem() {

        // Gets tabs from Shuffleboard
        ShuffleboardTab programmerBoard = Shuffleboard.getTab("Programmer Board");
        ShuffleboardTab driverBoard = Shuffleboard.getTab("Driver Board");

        // Gets the field information
        NetworkTable firstInfo = NetworkTableInstance.getDefault().getTable("FMSInfo");
        // Gets the team color from the field information

        // makes a team color chooser
        colorChooser.addOption(red, red);
        colorChooser.addOption(blue, blue);
        //driverBoard.add("Team Chooser", colorChooser).withWidget(BuiltInWidgets.kComboBoxChooser);

        xLimiter = new AccelerationLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        yLimiter = new AccelerationLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        turningLimiter = new AccelerationLimiter(DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond);

        camsDisabled = false;
        constantAim = false;

        xController = new PIDController(TargetPosConstants.kPDriveController, 0, 0);
        yController = new PIDController(TargetPosConstants.kPDriveController, 0, 0);
        thetaController = new PIDController(TargetPosConstants.kPAngleController, 0.0, 0.02);

        /*
         * Maybe the cause of the autonomous not working. When we call generate path in
         * command sequences the swerveSubsystem is
         * called 1st and then it tries to zero the heading! This might be a big
         * advancement
         */
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {

            }
        }).start();
    }

    public void zeroHeading() {
        gyro.setYaw(0);
    }

    public void setHeading(double yaw) {
        gyro.setYaw(yaw);
    }

    public double getHeading() {
        return gyro.getYaw().getValueAsDouble();
    }

    public boolean isAtPoint(Translation2d targetDrivePos) {
        return getPose().getTranslation().getDistance(targetDrivePos) //
                <= TargetPosConstants.kAcceptableDistanceError; //
    }

    public boolean isAtAngle(Rotation2d angle) {
        return Math.abs(odometer.getPoseMeters().getRotation().minus(angle).getDegrees()) //
                <= TargetPosConstants.kAcceptableAngleError;
    }

    // gets our current velocity relative to the y of the field
    public double getYSpeedFieldRel() {
        ChassisSpeeds temp = DriveConstants.kDriveKinematics.toChassisSpeeds(frontLeft.getState(),
                frontRight.getState(),
                backLeft.getState(), backRight.getState());
        temp = ChassisSpeeds.fromFieldRelativeSpeeds(temp, getRotation2d());

        return temp.vyMetersPerSecond;
    }

    public ChassisSpeeds getChassisSpeedsRobotRelative() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(chassisSpeeds, getRotation2d());
    }

    public double getXSpeedFieldRel() {
        ChassisSpeeds temp = DriveConstants.kDriveKinematics.toChassisSpeeds(frontLeft.getState(),
                frontRight.getState(),
                backLeft.getState(), backRight.getState());
        temp = ChassisSpeeds.fromFieldRelativeSpeeds(temp, getRotation2d());

        return temp.vxMetersPerSecond;
    }

    // gets our current velocity relative to the x of the robot (front/back)
    public double getXSpeedRobotRel() {
        ChassisSpeeds temp = DriveConstants.kDriveKinematics.toChassisSpeeds(frontLeft.getState(),
                frontRight.getState(),
                backLeft.getState(), backRight.getState());

        return temp.vxMetersPerSecond;
    }

    // gets our current velocity relative to the y of the robot (left/right)
    public double getYSpeedRobotRel() {
        ChassisSpeeds temp = DriveConstants.kDriveKinematics.toChassisSpeeds(frontLeft.getState(),
                frontRight.getState(),
                backLeft.getState(), backRight.getState());

        return temp.vyMetersPerSecond;
    }

    // gets our current angular velocity
    public double getRotationalSpeed() {
        ChassisSpeeds temp = DriveConstants.kDriveKinematics.toChassisSpeeds(frontLeft.getState(),
                frontRight.getState(),
                backLeft.getState(), backRight.getState());
        temp = ChassisSpeeds.fromFieldRelativeSpeeds(temp, getRotation2d());

        return temp.omegaRadiansPerSecond;
    }

    public Rotation2d getRotation2d() {
        return new Rotation2d().fromDegrees(getHeading());
    }

    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(getRotation2d(), getModulePositions(), pose);
    }

    public void initializeJoystickRunFromField() {
        xLimiter.setLimit(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        yLimiter.setLimit(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        turningLimiter.setLimit(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);

        xLimiter.reset(getXSpeedFieldRel());
        yLimiter.reset(getYSpeedFieldRel());
        turningLimiter.reset(getRotationalSpeed());
    }

    public void executeJoystickRunFromField(double xSpeedPercent, double ySpeedPercent, double thetaSpeedPercent) {

        runModulesFieldRelative(xSpeedPercent, ySpeedPercent, thetaSpeedPercent);
    }

    public void initializeDriveToPointAndRotate() {
        xLimiter.setLimit(TargetPosConstants.kForwardMaxAcceleration,
                TargetPosConstants.kBackwardMaxAcceleration);
        yLimiter.setLimit(TargetPosConstants.kForwardMaxAcceleration,
                TargetPosConstants.kBackwardMaxAcceleration);
        xLimiter.reset(getXSpeedFieldRel());
        yLimiter.reset(getYSpeedFieldRel());

        xController.setPID(TargetPosConstants.kPDriveController, 0, 0.002);
        xController.reset();
        yController.setPID(TargetPosConstants.kPDriveController, 0, 0.002);
        yController.reset();
        thetaController.setPID(TargetPosConstants.kPAngleController, 0, 0);
        thetaController.reset();
    }
    public void constantAim(){
        if(!constantAim)
            constantAim = true;
        else
            constantAim = false;
    }
    public boolean getConstantAim(){
        return constantAim;
    }

    public void executeDriveToPointAndRotate(Pose2d targetPosition) {
        double xSpeed = MathUtil.clamp(
                xController.calculate(getPose().getX(), targetPosition.getX()), -1, 1);
        double ySpeed = MathUtil.clamp(
                yController.calculate(getPose().getY(), targetPosition.getY()), -1, 1);

        Rotation2d angleDifference = odometer.getPoseMeters().getRotation().minus(targetPosition.getRotation());
        double turningSpeed = MathUtil.clamp(thetaController.calculate(angleDifference.getRadians(),
                0), -1, 1);
        turningSpeed *= TargetPosConstants.kMaxAngularSpeed;
        turningSpeed += Math.copySign(TargetPosConstants.kMinAngularSpeedRadians, turningSpeed);

        xSpeed = xLimiter.calculate(xSpeed * TargetPosConstants.kMaxSpeedMetersPerSecond);
        ySpeed = yLimiter.calculate(ySpeed * TargetPosConstants.kMaxSpeedMetersPerSecond);

        double unitCircleAngle = Math.atan2(ySpeed, xSpeed);
        xSpeed += Math.copySign(TargetPosConstants.kMinSpeedMetersPerSec, xSpeed) * Math.abs(Math.cos(unitCircleAngle));
        ySpeed += Math.copySign(TargetPosConstants.kMinSpeedMetersPerSec, ySpeed) * Math.abs(Math.sin(unitCircleAngle));
        
   
        xSpeed = -xSpeed;
        ySpeed = -ySpeed;
        
        
        runModulesFieldRelative(xSpeed, ySpeed, turningSpeed);
    }

    public void initializeRotateToAngle() {
        thetaController.setPID(TargetPosConstants.kPAngleController, 0, 0);
        thetaController.reset();
    }

    public void executeRotateToAngle(Rotation2d targetPosition) {
        Rotation2d angleDifference = odometer.getPoseMeters().getRotation().minus(targetPosition);

        double turningSpeed = MathUtil.clamp(thetaController.calculate(angleDifference.getRadians(),
                0), -1, 1) * TargetPosConstants.kMaxAngularSpeed;

        // turningSpeed += Math.copySign(TargetPosConstants.kMinAngularSpeedRadians, turningSpeed);

        runModulesFieldRelative(0, 0, turningSpeed);
    }

    public void runModulesFieldRelative(double xSpeed, double ySpeed, double turningSpeed) {
        // Converts robot speeds to speeds relative to field
        this.chassisSpeeds = chassisSpeeds;
        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed, ySpeed, turningSpeed, getRotation2d());

        // Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        // Output each module states to wheels
        setModuleStates(moduleStates);
    }

    public void runModulesRobotRelative(ChassisSpeeds chassisSpeeds) {
        // Converts robot speeds to speeds relative to field
        this.chassisSpeeds = chassisSpeeds;
        chassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
                chassisSpeeds, getRotation2d());

        // Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        // Output each module states to wheels
        setModuleStates(moduleStates);
    }

    protected StatusSignalJNI jni = new StatusSignalJNI();

    public double getPigeonRaw(int spn) {
        jni.network = gyro.getNetwork();
        jni.deviceHash = (int) gyro.getDeviceHash();
        jni.spn = spn;

        int error = jni.JNI_RefreshSignal(0);
        if (error != 0) {
            System.out.println("getPigeonRaw Error");
        }

        return jni.value;
    }

    @Override
    public void periodic() {
        odometer.update(getRotation2d(), getModulePositions());
        SmartDashboard.putNumber("Heading", getHeading());
        SmartDashboard.putNumber("odometry heading", odometer.getPoseMeters().getRotation().getDegrees());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
        SmartDashboard.putNumber("frontLeft Unfiltered Encoder", getFLAbsUnfilteredEncoder());
        SmartDashboard.putNumber("frontRight Unfiltered Encoder", getFRAbsUnfilteredEncoder());
        SmartDashboard.putNumber("BackLeft Unfiltered Encoder", getBLAbsUnfilteredEncoder());
        SmartDashboard.putNumber("BackRight Unfiltered Encoder", getBRAbsUnfilteredEncoder());
        SmartDashboard.putNumber("frontLeft Encoder", getFLAbsEncoder());
        SmartDashboard.putNumber("frontRight Encoder", getFRAbsEncoder());
        SmartDashboard.putNumber("BackLeft Encoder", getBLAbsEncoder());
        SmartDashboard.putNumber("BackRight Encoder", getBRAbsEncoder());

        Logger.recordOutput("Pigeon/RawYaw", getPigeonRaw(SpnValue.Pigeon2Yaw.value));
        Logger.recordOutput("Pigeon/RawPitch", getPigeonRaw(SpnValue.Pigeon2Pitch.value));
        Logger.recordOutput("Pigeon/RawRoll", getPigeonRaw(SpnValue.Pigeon2Roll.value));
        Logger.recordOutput("Pigeon/RawHeading", getPigeonRaw(SpnValue.Pigeon2AccumGyroZ.value));

        logOdometry();
        logPigeonState();
        logSwerveStates();
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void disableCams(){
        if(camsDisabled)
            camsDisabled = false;
        else
            camsDisabled = true;
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);

        logSwerveDesiredStates(desiredStates);
    }

    public SwerveModulePosition[] getModulePositions() {
        // Finds the position of each individual module based on the encoder values.
        SwerveModulePosition[] temp = { frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(),
                backRight.getPosition() };
        return temp;
    }

    // used for anything that requires team color.
    // this is housed in swerve subsystem since it uses it the most
    public static boolean isOnRed() {
        // gets the selected team color from the suffleboard
        Optional<Alliance> ally = DriverStation.getAlliance();
        if(ally.isPresent()){
            return ally.get() == Alliance.Red;
        }

        String choices = colorChooser.getSelected();
        return choices == "Red";
        // if no team selected on suffleboard, it will default to the field info
    }

    // send over shuffleboard values
    public double getFLAbsUnfilteredEncoder() {
        return frontLeft.getUnfilteredPosition();
    }
    public double getFLAbsEncoder(){
        return frontLeft.getAbsolutePosition();
    }

    public double getFRAbsUnfilteredEncoder() {
        return frontRight.getUnfilteredPosition();
    }
    public double getFRAbsEncoder(){
        return frontRight.getAbsolutePosition();
    }

    public double getBLAbsUnfilteredEncoder() {
        return backLeft.getUnfilteredPosition();
    }
    public double getBLAbsEncoder(){
        return backLeft.getAbsolutePosition();
    }

    public double getBRAbsUnfilteredEncoder() {
        return backRight.getUnfilteredPosition();
    }
    public double getBRAbsEncoder(){
        return backRight.getAbsolutePosition();
    }

    /* -- LOGGING -- */
    /*
     * Logging inputs and other random values with Advantage Kit
     * is extremely simple. Literally just run:
     * 
     * Logger.recordOutput(<name>, <value>);
     * 
     * <name> is a string containing the location were the value will be
     * saved (and thus appear in Advantage Scope). Such as, "Odometry/Location" will
     * save the value as "Location" under the "Odometry" Folder.
     * 
     * <value> can be of any rudimentary variable, Translation2d, Pose3d,
     * SwerveModuleState, and Mechanism2d. These are the values that will be logged.
     * All supported variable types are listed here:
     * https://github.com/Mechanical-Advantage/AdvantageKit/blob/main/docs/DATA-FLOW
     * .md#data-types
     * 
     * Logging can also be done using the @AutoLogOutput annotation.
     * More info here:
     * https://github.com/Mechanical-Advantage/AdvantageKit/blob/main/docs/RECORDING
     * -OUTPUTS.md#autologoutput-annotation
     * 
     * 
     */

    // Send the swerve modules' encoder positions to Advantage Kit
    public void logSwerveStates() {

        Logger.recordOutput("SwerveState", new SwerveModuleState[] {
                frontLeft.getState(),
                frontRight.getState(),
                backLeft.getState(),
                backRight.getState()
        });
    }

    // Send Swerve Desired Rotation and speed to Advantage Kit
    public void logSwerveDesiredStates(SwerveModuleState[] desiredStates) {
        Logger.recordOutput("DesiredSwerveState", desiredStates);
    }

    // Send Pigeon Rotation to Advantage Kit (useful for seeing robot rotation)
    public void logPigeonState() {
        Logger.recordOutput("PigeonGyro", gyro.getRotation2d());
    }

    // Send Odometry Position on field to Advantage Kit
    public void logOdometry() {
        Logger.recordOutput("Odometry/Location", odometer.getPoseMeters());
    }
}
