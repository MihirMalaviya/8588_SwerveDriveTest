// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.ScoringArea;
import frc.robot.subsystems.Wrist;

/** An example command that uses an example subsystem. */
public class WristAimbot extends Command {
  private final Wrist m_wrist;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public WristAimbot() {
    m_wrist = RobotContainer.m_wrist;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_wrist.setWristGoalCommand(
        RobotContainer.scoringArea == ScoringArea.SPEAKER 
        ? RobotContainer.speakerLookupTable.getAngleFromDistance(RobotContainer.distanceFromTarget) 
        : RobotContainer.ampLookupTable.getAngleFromDistance(RobotContainer.distanceFromTarget));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}