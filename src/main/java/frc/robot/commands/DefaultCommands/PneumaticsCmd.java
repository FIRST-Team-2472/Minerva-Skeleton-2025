package frc.robot.commands.DefaultCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PneumaticsSubsystem;

public class PneumaticsCmd extends Command{

    PneumaticsSubsystem pneumaticsSubsystem;

    public PneumaticsCmd(PneumaticsSubsystem pneumaticsSubsystem){
        this.pneumaticsSubsystem = pneumaticsSubsystem;
        addRequirements(pneumaticsSubsystem);
    }
}
