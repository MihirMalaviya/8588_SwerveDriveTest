// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.PhotonCamera;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.AimbotDrive;
// import frc.robot.commands.DefaultDrive;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.PurgeCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.StopCommand;
import frc.robot.commands.WristAimbot;
import frc.robot.subsystems.Indexing;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Wrist;
import frc.robot.utils.LookupTable;
import frc.robot.auto.TestAuto;
// import frc.robot.auto.AutoController.autoRoutines;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  public static enum ScoringArea {
    SPEAKER,
    AMP//,
    // TRAP
  }

  public static ScoringArea scoringArea = ScoringArea.AMP;
  
  // hardcoded lookup tables
  public static final LookupTable speakerLookupTable = new LookupTable(
    new double[] {1, 2, 3, 4, 5}, // radians
    new double[] {1, 2, 3, 4, 5}  // meters
  );
  public static final LookupTable ampLookupTable = new LookupTable(
    new double[] {1, 2, 3, 4, 5}, // radians
    new double[] {1, 2, 3, 4, 5}  // meters
  );

  // The robot's subsystems and commands are defined here...
  public static final SwerveSubsystem m_Swerb = new SwerveSubsystem();
  public static final PhotonCamera m_camera = new PhotonCamera(VisionConstants.CAMERA_NAME);
  public static final PhotonCamera m_noteCamera = new PhotonCamera("notes");

  public static final Intake m_intake = new Intake();
  public static final Indexing m_indexing = new Indexing();
  public static final Shooter m_shooter = new Shooter();
  public static final Wrist m_wrist = new Wrist();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  // public static CommandJoystick commandJoystick = new CommandJoystick(OperatorConstants.kDriverControllerPort);
  public static CommandXboxController driverXbox = new CommandXboxController(OperatorConstants.kDriverControllerPort);

  public static final IntakeCommand intakeCommand = new IntakeCommand();
  public static final ShootCommand shootCommand = new ShootCommand();
  public static final PurgeCommand purgeCommand = new PurgeCommand();
  public static final StopCommand stopCommand = new StopCommand();
  public static final AimbotDrive aimbotDrive = new AimbotDrive();
  
  public static DoubleLogEntry shotDistance; // meters
  public static DoubleLogEntry shotAngle; // radians

  public static double distanceFromTarget = 1.0; // meters
  public static double shootAngle = 0.0; // radians

  public static boolean isRedAlliance = true;

  public static SendableChooser<String> autoSelector;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    // Starts recording to data log
    DataLogManager.start();

    // Set up custom log entries
    DataLog log = DataLogManager.getLog();
    shotDistance = new DoubleLogEntry(log, "/shots/distance");
    shotAngle = new DoubleLogEntry(log, "/shots/angle");

    var ally = DriverStation.getAlliance();
    isRedAlliance = ally.isPresent() 
        ? ally.get() == Alliance.Red 
        : true;
  
    // // create auto selector with each enum option
    // autoSelector = new SendableChooser<>();
    // autoSelector.setDefaultOption(autoRoutines.values()[0].toString(), autoRoutines.values()[0].toString());
    // for (int i = 1; i < autoRoutines.values().length; i++) {
    //     autoSelector.addOption(autoRoutines.values()[i].toString(), autoRoutines.values()[i].toString());
    // }

    // SmartDashboard.putData("Auton Selector", autoSelector);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // m_Swerb.setDefaultCommand(new DefaultDrive());

    // m_Swerb.setDefaultCommand(aimbotDrive);
    m_wrist.setDefaultCommand(new WristAimbot());

    driverXbox.y()
      .onTrue(new InstantCommand(() -> m_Swerb.zeroYaw()));

    driverXbox.a()
      .onTrue(intakeCommand);
    
    driverXbox.b()
      .onTrue(shootCommand);
    
    driverXbox.x()
      .onTrue(purgeCommand);
    
    // // for intake (manual)
    // driverXbox.a()
    // .onTrue(Commands.runOnce(() -> {
    //   m_intake.intake();
    //   m_indexing.intake();
    // }))
    // .onFalse(Commands.runOnce(() -> {
    //   m_intake.stop();
    //   m_indexing.stop();
    // }));

    // // for shooting (manual)
    // driverXbox.b()
    // .onTrue(Commands.runOnce(() -> {
    //   m_indexing.shoot();
    //   m_shooter.shoot();
    // }))
    // .onFalse(Commands.runOnce(() -> {
    //   m_indexing.stop();
    //   m_shooter.stop();
    // }));
    
    // for aiming
    // driverXbox.leftBumper().onTrue(m_wrist.incrementUp());
    // driverXbox.rightBumper().onTrue(m_wrist.incrementDown());

    /*
     * CONTROLLER CHANNELS
     * Analog:
     * 0 - Left X
     * 1 - Left Y
     * 4 - Right X
     * 
     * Digital:
     * 4 - Y Button
     */
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // return new InstantCommand(); // this is the equivalent of "do nothing"
    return new TestAuto();
  }
}
