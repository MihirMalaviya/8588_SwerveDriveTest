// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;

/** An example command that uses an example subsystem. */
public class FollowPathCommand extends Command {

  String pathName;
  public FollowPathCommand(String pathName) {
    this.pathName = pathName;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_Swerb);
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.m_Swerb.initPath(pathName, RobotContainer.isRedAlliance);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.m_Swerb.followPath();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
