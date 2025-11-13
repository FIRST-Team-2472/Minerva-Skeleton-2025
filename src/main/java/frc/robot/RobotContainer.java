package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ArmMotorsConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.*;
import frc.robot.commands.DefaultCommands.*;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
    private final String SPtwoNtwoNone = "3 notes in speaker Speaker 2 Note 2 - 1 - TESTED", SPtwoNtwo = "2 notes in speaker Seaker 2 Note 2 - TESTED",
            SPtwoNoneNtwoNthree = "4 notes in speaker from Speaker 2 Notes 1 - 2 - 3", SPtwoNthreeNtwoNoneNfour = "4 in speaker Speaker 2 Notes 3 - 2 - 1 - 4",
            SPtwoNoneNfour = "3 in speaker Speaker 2 Notes 2 - 1 pick up 4", SPtwoNtwoNfour = "3 in speaker Speaker 2 Notes 2 - 4",
            SPoneNoneNfourRSPone = "3 in speaker Speaker one Notes 1 - 4", SPthreeNthreeNeightNseven = "3 in speaker Speaker 3 Notes 3 - 8 - 7",
            SPthreeNfourNfive = "3 in speaker Speaker 3 Notes 4 - 5", SPthreeNfiveNfour = "3 in speaker Speaker 3 Notes 5 - 4",
            SPthreeNthree = "2 in speaker Speaker 3 Note 3", SPoneNone = "2 in speaker Speaker 1 Note 1",
            SPthreeNeightNseven = "3 in speaker Speaker 3 Notes 8 - 7", SpThreeNThreeNEight = "3 in speaker out of the way stage side", test = "test",
            justShoot = "just shoot";

    private String m_autoSelected;
    private final SendableChooser<String> m_chooser = new SendableChooser<>();

    private final CommandSequences commandSequences = new CommandSequences();
    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();


    //private final Limelights limelights = new Limelights(swerveSubsystem, armSubsystem);
    XboxController xbox = new XboxController(OperatorConstants.kXboxControllerPort);
    public static Joystick leftJoystick = new Joystick(OperatorConstants.kLeftJoyPort);
    public static Joystick rightJoystick = new Joystick(OperatorConstants.kRightJoyPort);



    public RobotContainer() {
        swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(swerveSubsystem,
                ()-> xbox.getLeftY(),
                ()-> xbox.getLeftX(),
                ()-> -xbox.getRightX(),
                ()-> false
        ));

        configureBindings();

        m_chooser.addOption(SPtwoNtwoNone, SPtwoNtwoNone);
        m_chooser.addOption(SPtwoNtwo, SPtwoNtwo);
        m_chooser.addOption(SPtwoNoneNtwoNthree, SPtwoNoneNtwoNthree);
        m_chooser.addOption(SPtwoNthreeNtwoNoneNfour, SPtwoNthreeNtwoNoneNfour);
        m_chooser.addOption(SPtwoNoneNfour, SPtwoNoneNfour);
        m_chooser.addOption(SPtwoNtwoNfour, SPtwoNtwoNfour);
        m_chooser.addOption(SPoneNoneNfourRSPone, SPoneNoneNfourRSPone);
        m_chooser.addOption(SPthreeNthreeNeightNseven, SPthreeNthreeNeightNseven);
        m_chooser.addOption(SPthreeNfourNfive, SPthreeNfourNfive);
        m_chooser.addOption(SPthreeNfiveNfour, SPthreeNfiveNfour);
        m_chooser.addOption(SPthreeNthree, SPthreeNthree);
        m_chooser.addOption(SPoneNone, SPoneNone);
        m_chooser.addOption(SPthreeNeightNseven, SPthreeNeightNseven);
        m_chooser.addOption(SpThreeNThreeNEight, SpThreeNThreeNEight);
        m_chooser.addOption(test, test);
        m_chooser.addOption(justShoot, justShoot);

        ShuffleboardTab driverBoard = Shuffleboard.getTab("Driver Board");
        driverBoard.add("Auto choices", m_chooser).withWidget(BuiltInWidgets.kComboBoxChooser);
        driverBoard.addCamera("Limelight Stream Intake", "limelight_intake", "mjpg:http://limelight-intake.local:5800").withSize(4,4);
        driverBoard.addCamera("Limelight Stream Shooter", "limelight_shooter", "mjpg:http://limelight-shooter.local:5800").withSize(4,4);

        //warning a name change will break auto paths because pathplanner will not update it

        System.out.println("team " + SwerveSubsystem.isOnRed());
    }

    private void configureBindings() {
        new JoystickButton(xbox, 7).onTrue(new InstantCommand(swerveSubsystem :: zeroHeading));
        new JoystickButton(rightJoystick, 13).onTrue(new InstantCommand(swerveSubsystem :: disableCams));

    }

    public Command getAutonomousCommand() {
        System.out.println("Autos Begun");

        return new EvaluateGyroCommand(swerveSubsystem, 5, 10);
    }

    public void logSwerve() {

        SmartDashboard.putNumber("Heading", swerveSubsystem.getHeading());
        SmartDashboard.putString("Robot Location", swerveSubsystem.getPose().getTranslation().toString());
        SmartDashboard.putNumber("frontLeft Encoder", swerveSubsystem.getFLAbsEncoder());
        SmartDashboard.putNumber("frontRight Encoder", swerveSubsystem.getFRAbsEncoder());
        SmartDashboard.putNumber("BackLeft Encoder", swerveSubsystem.getBLAbsEncoder());
        SmartDashboard.putNumber("BackRight Encoder", swerveSubsystem.getBRAbsEncoder());
        SmartDashboard.putNumber("Rotation", swerveSubsystem.getRotation2d().getDegrees());
        //SmartDashboard.putNumber("Arm Encoder", armSubsystem.getAbsoluteEncoder());
    }
}
