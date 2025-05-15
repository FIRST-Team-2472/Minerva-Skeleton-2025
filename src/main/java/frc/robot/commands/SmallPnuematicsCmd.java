package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PneumaticsSubsystem; 

public class SmallPnuematicsCmd extends Command{

    PneumaticsSubsystem pneumaticsSubsystem;
    private Timer timer;

    public SmallPnuematicsCmd(PneumaticsSubsystem pneumaticsSubsystem){
        this.pneumaticsSubsystem = pneumaticsSubsystem;
        addRequirements(pneumaticsSubsystem);
        timer = new Timer();
    }

    @Override
    public void initialize() {
        timer.restart();
        if(!pneumaticsSubsystem.bigLeftExtended && !pneumaticsSubsystem.smallLeftExtended){
            pneumaticsSubsystem.doubleSolenoidSmallLeft.set(DoubleSolenoid.Value.kForward);
            pneumaticsSubsystem.doubleSolenoidSmallRight.set(DoubleSolenoid.Value.kForward);

            pneumaticsSubsystem.smallLeftExtended = true;
            pneumaticsSubsystem.smallRightExtended = true;
        } else if(!pneumaticsSubsystem.bigLeftExtended){
            pneumaticsSubsystem.doubleSolenoidSmallLeft.set(DoubleSolenoid.Value.kReverse);
            pneumaticsSubsystem.doubleSolenoidSmallRight.set(DoubleSolenoid.Value.kReverse);
            
            pneumaticsSubsystem.smallLeftExtended = false;
            pneumaticsSubsystem.smallRightExtended = false;
        }
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        pneumaticsSubsystem.doubleSolenoidSmallLeft.set(DoubleSolenoid.Value.kOff);
		pneumaticsSubsystem.doubleSolenoidSmallRight.set(DoubleSolenoid.Value.kOff);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(1);
    }
    
}
