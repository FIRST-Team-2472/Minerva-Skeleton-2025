package frc.robot.subsystems.swerveExtras;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;


public class SwerveModule {
    
    private final SparkMax driveMotor;
    private final SparkMax turningMotor;

    private CANcoder absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffset;
    private final PIDController turningPidController;

    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed, 
            int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed){
        this.absoluteEncoderOffset = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        
        absoluteEncoder = new CANcoder(absoluteEncoderId);

        driveMotor = new SparkMax(driveMotorId, MotorType.kBrushless);
        turningMotor = new SparkMax(turningMotorId, MotorType.kBrushless);

        SparkMaxConfig sparkConfig = new SparkMaxConfig();
        sparkConfig.smartCurrentLimit(35);
        sparkConfig.idleMode(IdleMode.kBrake); //they didnt move on brake mode, we also dont really need it on that mode
        driveMotor.configure(sparkConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        turningMotor.configure(sparkConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        // lets us use a PID system for the turning motor
        turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        // tells the PID controller that our motor can go from -PI to PI (it can rotate
        // continously)
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);
        
        resetEncoders();
    }
    

    public double getDrivePosition(){
        return driveMotor.getEncoder().getPosition() * ModuleConstants.kDriveEncoderRot2Meter;
    }

    public double getTurningPosition(){
        return turningMotor.getEncoder().getPosition() * ModuleConstants.kTurningEncoderRot2Rad;
    }

    public double getDriveVelocity(){
        return driveMotor.getEncoder().getVelocity() * ModuleConstants.kDriveEncoderRPM2MeterPerSec;
    }

    public double getTurningVelocity(){
        return turningMotor.getEncoder().getVelocity() * ModuleConstants.kTurningEncoderRPM2RadPerSec;
    }
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), Rotation2d.fromDegrees(getAbsolutePosition()));
    }
    public double getAbsoluteEncoder(){ // this is used for shuffleboard
        return absoluteEncoder.getAbsolutePosition().getValueAsDouble();
    }
    public double getUnfilteredPosition(){
        double angle = absoluteEncoder.getAbsolutePosition().getValueAsDouble();
        return angle;
    }
    public double getAbsolutePosition() {
        // converts from (-.5, .5) to (-180, 180)
        double angle = 360 * absoluteEncoder.getAbsolutePosition().getValueAsDouble();
        //angle -= absoluteEncoderOffset;
        angle *= absoluteEncoderReversed ? -1 : 1;
        
        return angle;
    }
    

    public void resetEncoders(){
        driveMotor.getEncoder().setPosition(0);
        turningMotor.getEncoder().setPosition(getAbsolutePosition());
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(getDriveVelocity(), Rotation2d.fromDegrees(getAbsolutePosition()));
    }

    public void setDesiredState(SwerveModuleState state){
        if(Math.abs(state.speedMetersPerSecond) < 0.001){
            stop(); // keeps it from flipping back forward when not moving
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle); // makes it so we can reverse the wheels instead of spinning 180
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        turningMotor.set(turningPidController.calculate(getState().angle.getRadians(), state.angle.getRadians()));
    }

    public void stop(){
        driveMotor.set(0);
        turningMotor.set(0);
    }
}
