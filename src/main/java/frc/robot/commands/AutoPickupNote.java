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
public class AutoPickupNote extends Command {
  // PID constants should be tuned per robot
  PIDController forwardController = new PIDController(DriveConstants.kLinearP, DriveConstants.kLinearI, DriveConstants.kLinearD);
  PIDController turnController = new PIDController(DriveConstants.kAngularP, DriveConstants.kAngularI, DriveConstants.kAngularD);

  // i heard trapezoidprofile was suggested for positions so maybe we can switch to that
  // TrapezoidProfile xRateLimiter = new TrapezoidProfile(new Constraints(1.0, 1.0));
  // TrapezoidProfile yRateLimiter = new TrapezoidProfile(new Constraints(1.0, 1.0));
  // TrapezoidProfile wRateLimiter = new TrapezoidProfile(new Constraints(1.0, 1.0));

  // seperate in case we decide 
  SlewRateLimiter xRateLimiter = new SlewRateLimiter(DriveConstants.kXYSlewRate);
  SlewRateLimiter yRateLimiter = new SlewRateLimiter(DriveConstants.kXYSlewRate);
  SlewRateLimiter wRateLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);

  PhotonCamera m_camera = RobotContainer.m_noteCamera;
  CommandXboxController m_controller = RobotContainer.driverXbox;

  double range = 0.0; // meters

  public AutoPickupNote() {

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_Swerb);
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double forwardSpeed = 0.0;
    double rotationSpeed = 0.0;

    double vx = 0.0;
    double vy = 0.0;
    double vw = 0.0;

    // Vision-alignment mode
    // Query the latest result from PhotonVision
    var result = m_camera.getLatestResult();
    PhotonTrackedTarget bestTarget = result.getBestTarget();
    
    if (bestTarget != null) {

      // First calculate range
      range = PhotonUtils.calculateDistanceToTargetMeters(
                      VisionConstants.CAMERA_HEIGHT_METERS,
                      0.0, // height
                      VisionConstants.CAMERA_PITCH_RADIANS,
                      Units.degreesToRadians(result.getBestTarget().getPitch()));

      // Use this range as the measurement we give to the PID controller.
      // (This forwardSpeed must be positive to go forward.)
      forwardSpeed = forwardController.calculate(range, 0.0);

      // Also calculate angular power
      // (This rotationSpeed must be positive to turn counter-clockwise.)
      rotationSpeed = turnController.calculate(result.getBestTarget().getYaw(), 0);

      vx = xRateLimiter.calculate(Math.cos(forwardSpeed));
      vy = yRateLimiter.calculate(Math.sin(forwardSpeed));
      vw = wRateLimiter.calculate(rotationSpeed);
        
    } 
    // else { // no target found
    //    // rn this just stops the robot but we might want it to look at some target position or sm

    //   vx = 0.0;
    //   vy = 0.0;
    //   vw = 0.0;
      
    // }

    // Apply slew rate limits
    vx = xRateLimiter.calculate(vx);
    vy = yRateLimiter.calculate(vy);
    vw = wRateLimiter.calculate(vw);

    SmartDashboard.putNumber("Vx", vx);
    SmartDashboard.putNumber("Vy", vy);
    SmartDashboard.putNumber("Vw", vw);
    
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(vy, vx, vw, Rotation2d.fromDegrees(-RobotContainer.m_Swerb.getYaw()));
    RobotContainer.m_Swerb.drive(speeds);

  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
