package frc.robot.subsystems.ArmSubsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmMotorsConstants.*;
import frc.robot.Constants.SensorConstants;

public class IntakeMotorSubsystem extends SubsystemBase {
    private SparkMax pushMotor = new SparkMax(PushMotor.kPushMotorId, MotorType.kBrushless);
    private SparkMax intakeTopMotor = new SparkMax(IntakeMotors.kTopIntakeMotorId, MotorType.kBrushless);
    private SparkMax intakeBottomMotor = new SparkMax(IntakeMotors.kBottomIntakeMotorId, MotorType.kBrushless);
    DigitalInput photoElectricSensor = new DigitalInput(SensorConstants.kPhotoElectricSensorID);

    public IntakeMotorSubsystem() {

        // make sure all of them have the same settings in case we grabbed one with presets
        SparkMaxConfig config = new SparkMaxConfig();
            config.smartCurrentLimit(35);
            config.idleMode(IdleMode.kBrake);

        pushMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        intakeTopMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        intakeBottomMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {}

    /**
     * Controls the speed the pushing or feeding motors are running
     * 
     * @param motorSpeed speed to run feeding motors at (between -1 & 1)
     */
    public void runPushMotor(double motorSpeed) {
        pushMotor.set(motorSpeed);
    }

    /**
     * Controls the speed the intake motors are running
     * 
     * @param motorSpeed speed to run intake motors at (between -1 & 1)
     */
    public void runIntakeMotors(double motorSpeed) {
        intakeTopMotor.set(motorSpeed);
        intakeBottomMotor.set(-motorSpeed);
    }

    /**
     * Gets whither the photo electric sensor on the intake chamber sees a not or not
     * 
     * @return whither the sensor sees a note
     */
    public boolean getPhotoElectricSensor(){
        return !photoElectricSensor.get();
    }
    
}
