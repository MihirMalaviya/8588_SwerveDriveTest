// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DefaultDrive;
import frc.robot.subsystems.Indexing;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Wrist;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static final SwerveSubsystem m_Swerb = new SwerveSubsystem();

  public static final Intake m_intake = new Intake();
  public static final Indexing m_indexing = new Indexing();
  public static final Shooter m_shooter = new Shooter();
  public static final Wrist m_wrist = new Wrist();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  // public static CommandJoystick commandJoystick = new CommandJoystick(OperatorConstants.kDriverControllerPort);
  public static CommandXboxController driverXbox = new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
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
    m_Swerb.setDefaultCommand(new DefaultDrive());
    // commandJoystick.button(4)
    driverXbox.y()
      .onTrue(new InstantCommand(() -> m_Swerb.zeroYaw()));
    
    // for intake (manual)
    driverXbox.a()
    .onTrue(Commands.runOnce(() -> {
      m_intake.intake();
      m_indexing.intake();
    }))
    .onFalse(Commands.runOnce(() -> {
      m_intake.stop();
      m_indexing.stop();
    }));

    // for shooting (manual)
    driverXbox.b()
    .onTrue(Commands.runOnce(() -> {
      m_indexing.shoot();
      m_shooter.shoot();
    }))
    .onFalse(Commands.runOnce(() -> {
      m_indexing.stop();
      m_shooter.stop();
    }));
    
    // for aiming
    driverXbox.leftBumper().onTrue(m_wrist.incrementUp());
    driverXbox.rightBumper().onTrue(m_wrist.incrementDown());

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
    // TODO IMPLEMENT AN AUTON COMMAND!!!
    return new InstantCommand(); // this is the equivalent of "do nothing"
  }
}
