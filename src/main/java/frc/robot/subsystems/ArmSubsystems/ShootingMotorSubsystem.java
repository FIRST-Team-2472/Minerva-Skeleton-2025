package frc.robot.subsystems.ArmSubsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmMotorsConstants.*;

public class ShootingMotorSubsystem extends SubsystemBase {
    private static SparkMax shooterTopMotor = new SparkMax(ShooterMotors.kTopShooterMotorId, MotorType.kBrushless);
    private SparkMax shooterBottomMotor = new SparkMax(ShooterMotors.kBottomShooterMotorId, MotorType.kBrushless);
    private PIDController shooterPID = new PIDController(0.5, 0.2, 0);
    public boolean constantAim;

    public ShootingMotorSubsystem() {

        // make sure all of them have the same settings in case we grabbed one with presets
        SparkMaxConfig config = new SparkMaxConfig();
            config.smartCurrentLimit(35);
            config.idleMode(IdleMode.kCoast);
        shooterBottomMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        shooterTopMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        constantAim = false;

    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter speed", getShooterSpeed());
    }

    /**
     * Controls the speed the shooting motors (flywheels) are running
     * 
     * @param motorSpeed speed to run shooting motors at (between -1 & 1)
     */
    public void runShooterMotors(double motorSpeed) {
        shooterTopMotor.set(-motorSpeed);
        shooterBottomMotor.set(motorSpeed);
    }
    
    public void runShooterMotorsWithKP(double targetRpm) {
        SmartDashboard.putNumber("shooter pid", PitchMotorSubsystem.clamp(shooterPID.calculate(getShooterSpeed(), -targetRpm), 0, 1));
        runShooterMotors(PitchMotorSubsystem.clamp(shooterPID.calculate(getShooterSpeed(), -targetRpm), 0, 1));
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

    /**
     * Gets the shooting motors' (flywheels') current speed
     * 
     * @return the shooting motors' speed
     */
    public double getShooterSpeed(){
        return shooterTopMotor.getEncoder().getVelocity();
    }
}
