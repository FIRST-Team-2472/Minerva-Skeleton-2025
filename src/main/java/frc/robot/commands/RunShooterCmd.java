package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystems.ShootingMotorSubsystem;

public class RunShooterCmd extends Command {
    double rpm = 0;
    ShootingMotorSubsystem shootingSubsystem;
    Timer timer = new Timer();

    public RunShooterCmd(ShootingMotorSubsystem shootingSubsystem, double rpm) {
        this.rpm = -rpm;
        this.shootingSubsystem = shootingSubsystem;

        addRequirements(shootingSubsystem);
    }

    @Override
    public void initialize() {
        timer.restart();
    }

    @Override
    public void execute() {
        shootingSubsystem.runShooterMotorsWithKP(rpm);
    }

    @Override
    public void end(boolean interrupted) {
        shootingSubsystem.runShooterMotors(0.0);
    }
    
    @Override
    public boolean isFinished() {
        return false;
        // return (Math.abs(shootingSubsystem.getShooterSpeed() - rpm) < 50) | timer.hasElapsed(5);
    }
}