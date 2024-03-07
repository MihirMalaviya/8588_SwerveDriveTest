// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import static frc.robot.Constants.VisionConstants;

import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.RobotContainer.ScoringArea;
import frc.robot.RobotContainer;

/** An example command that uses an example subsystem. */
public class PickupNote extends Command {
  // // Change this to match the name of your camera
  // PhotonCamera camera = new PhotonCamera("photonvision");

  // PID constants should be tuned per robot
  PIDController forwardController = new PIDController(DriveConstants.kLinearP, DriveConstants.kLinearI, DriveConstants.kLinearD);
  PIDController turnController = new PIDController(DriveConstants.kAngularP, DriveConstants.kAngularI, DriveConstants.kAngularD);

  // i heard trapezoidprofile was suggested for positions so maybe we can switch to that
  // TrapezoidProfile xRateLimiter = new TrapezoidProfile(new Constraints(1.0, 1.0));
  // TrapezoidProfile yRateLimiter = new TrapezoidProfile(new Constraints(1.0, 1.0));
  // TrapezoidProfile wRateLimiter = new TrapezoidProfile(new Constraints(1.0, 1.0));

  SlewRateLimiter xRateLimiter = new SlewRateLimiter(2.0 * DriveConstants.kXYSlewRate);
  SlewRateLimiter yRateLimiter = new SlewRateLimiter(2.0 * DriveConstants.kXYSlewRate);
  SlewRateLimiter wRateLimiter = new SlewRateLimiter(2.0 * DriveConstants.kRotationalSlewRate);

  PhotonCamera m_camera = RobotContainer.m_camera;
  CommandXboxController m_controller = RobotContainer.driverXbox;

  // double shootAngle = 0.0; // radians
  double range = 1.0; // meters
  double goalRange = 2.0; // meters

  public PickupNote() {

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_Swerb);
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Vision-alignment mode
    // Query the latest result from PhotonVision
    var result = m_camera.getLatestResult();

    double vx = 0.0;
    double vy = 0.0;
    double vw = 0.0;
 
    boolean targetFound = false;
    
    if (result.hasTargets()) {
      PhotonTrackedTarget bestTarget = result.getBestTarget();
        
      double forwardSpeed  = 0;
      double rotationSpeed = 0;

      // Use this range as the measurement we give to the PID controller.
      // (This forwardSpeed must be positive to go forward.)
      forwardSpeed = 0.1;

      // Also calculate angular power
      // (This rotationSpeed must be positive to turn counter-clockwise.)
      rotationSpeed = turnController.calculate(result.getBestTarget().getYaw(), 0);

      vx = xRateLimiter.calculate(Math.cos(forwardSpeed));
      vy = yRateLimiter.calculate(Math.sin(forwardSpeed));
      vw = wRateLimiter.calculate(rotationSpeed);
    } 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
