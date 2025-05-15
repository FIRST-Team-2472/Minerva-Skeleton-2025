package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystems.ShootingMotorSubsystem;

public class HoldShooterCmd extends Command{

    Timer overideTimer;
    ShootingMotorSubsystem shootingMotorSubsystem;

    public HoldShooterCmd(ShootingMotorSubsystem shootingMotorSubsystem){
        this.shootingMotorSubsystem = shootingMotorSubsystem;
        addRequirements(shootingMotorSubsystem);
        overideTimer = new Timer();
    }

    @Override
    public void initialize() {
        overideTimer.restart();
        System.out.println("shooter comand started");
    }

    @Override
    public void execute(){
        shootingMotorSubsystem.runShooterMotorsWithKP(4000);
        SmartDashboard.putBoolean("shooterRunning", isFinished());
    }

    @Override
    public void end(boolean interrupted) {
        shootingMotorSubsystem.runShooterMotors(0);
        System.out.println("shooter comand ended" +overideTimer.get());
        if(interrupted)
            System.out.println("interruped");
     }

     @Override
    public boolean isFinished() {
        return overideTimer.hasElapsed(15);
    }
}
