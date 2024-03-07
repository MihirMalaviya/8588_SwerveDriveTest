// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
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
import frc.robot.Constants.VisionConstants;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.ScoringArea;

/** An example command that uses an example subsystem. */
public class AimbotDrive extends Command {
  // // Change this to match the name of your camera
  // PhotonCamera camera = new PhotonCamera("photonvision");

  // PID constants should be tuned per robot
  PIDController forwardController = new PIDController(DriveConstants.kLinearP, DriveConstants.kLinearI, DriveConstants.kLinearD);
  PIDController turnController = new PIDController(DriveConstants.kAngularP, DriveConstants.kAngularI, DriveConstants.kAngularD);

  // i heard trapezoidprofile was suggested for positions so maybe we can switch to that
  // TrapezoidProfile xRateLimiter = new TrapezoidProfile(new Constraints(1.0, 1.0));
  // TrapezoidProfile yRateLimiter = new TrapezoidProfile(new Constraints(1.0, 1.0));
  // TrapezoidProfile wRateLimiter = new TrapezoidProfile(new Constraints(1.0, 1.0));

  SlewRateLimiter xRateLimiterDriving = new SlewRateLimiter(DriveConstants.kXYSlewRate);
  SlewRateLimiter yRateLimiterDriving = new SlewRateLimiter(DriveConstants.kXYSlewRate);
  SlewRateLimiter wRateLimiterDriving = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
  
  // seperate in case we decide 
  SlewRateLimiter xRateLimiterAimbot = new SlewRateLimiter(2.0 * DriveConstants.kXYSlewRate);
  SlewRateLimiter yRateLimiterAimbot = new SlewRateLimiter(2.0 * DriveConstants.kXYSlewRate);
  SlewRateLimiter wRateLimiterAimbot = new SlewRateLimiter(2.0 * DriveConstants.kRotationalSlewRate);

  PhotonCamera m_camera = RobotContainer.m_camera;
  CommandXboxController m_controller = RobotContainer.driverXbox;
  
  // double shootAngle = 0.0; // radians
  double range = 1.0; // meters
  double goalRange = 2.0; // meters

  public AimbotDrive() {

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

    Optional<Alliance> alliance = DriverStation.getAlliance();
    boolean isRed = alliance.isPresent() 
      ? alliance.get() == Alliance.Red 
      : true; // this shouldnt happen
 
    boolean targetFound = false;

    if (m_controller.getHID().getAButton()) { // if A button is held down
      if (result.hasTargets()) { // if apriltags are detected by the camera
        PhotonTrackedTarget bestTarget = result.getBestTarget();
        int bestTargetId = bestTarget.getFiducialId();
        
        if (isRed) { // we are in red alliance
          if (bestTargetId >= 3 && bestTargetId <= 5) { // only if they are the ones we are looking for
            targetFound = true;
          } else {
            var targets = result.getTargets();

            for (PhotonTrackedTarget target : targets) {
              int targetId = target.getFiducialId();
              if (targetId >= 3 && targetId <= 5) {
                bestTarget = target;
                targetFound = true;
                break;
              }
            }
          }
        } else { // we are in blue alliance
          if (bestTargetId >= 6 && bestTargetId <= 8) { // only if they are the ones we are looking for
            targetFound = true;
          } else {
            var targets = result.getTargets();

            for (PhotonTrackedTarget target : targets) {
              int targetId = target.getFiducialId();
              if (targetId >= 3 && targetId <= 5) {
                bestTarget = target;
                targetFound = true;
                break;
              }
            }
          }
        }
      }
    }
    
    if (targetFound) {
      PhotonTrackedTarget bestTarget = result.getBestTarget();
      int bestTargetId = bestTarget.getFiducialId();
        
      double forwardSpeed  = 0;
      double rotationSpeed = 0;

      double targetHeight = Units.inchesToMeters(53.38); // meters (initialized so code doesnt error)

      switch (bestTargetId) {
        case 3: // speaker
        case 4:
        case 7:
        case 8:
          RobotContainer.scoringArea = ScoringArea.SPEAKER;
          targetHeight = Units.inchesToMeters(57.13);
          goalRange = 2.0;

          
          break;
        case 5: // amp
        case 6:
        RobotContainer.scoringArea = ScoringArea.AMP;
          targetHeight = Units.inchesToMeters(53.38);
          break;
        // case 1: // trap
        // case 2:
        // case 9:
        // case 10:
        //   scoringArea = ScoringArea.TRAP;
        //   targetHeight = Units.inchesToMeters(53.38);
        //   break;

        default: // this shouldnt happen...
          break;
      }
      
      // First calculate range
      range = PhotonUtils.calculateDistanceToTargetMeters(
                      VisionConstants.CAMERA_HEIGHT_METERS,
                      targetHeight,
                      VisionConstants.CAMERA_PITCH_RADIANS,
                      Units.degreesToRadians(result.getBestTarget().getPitch()));

      // Use this range as the measurement we give to the PID controller.
      // (This forwardSpeed must be positive to go forward.)
      forwardSpeed = 
          RobotContainer.scoringArea == ScoringArea.SPEAKER 
              ? -forwardController.calculate(range, goalRange) 
              : 0.0;

      // Also calculate angular power
      // (This rotationSpeed must be positive to turn counter-clockwise.)
      rotationSpeed = turnController.calculate(RobotContainer.scoringArea == ScoringArea.SPEAKER  
          ? PhotonUtils.getYawToPose(RobotContainer.m_Swerb.getPose(), isRed ? new Pose2d(RobotContainer.m_Swerb.redSpeakerPos.getX(),RobotContainer.m_Swerb.redSpeakerPos.getY(), new Rotation2d())
                                                                             : new Pose2d(RobotContainer.m_Swerb.blueSpeakerPos.getX(),RobotContainer.m_Swerb.blueSpeakerPos.getY(), new Rotation2d())).getRadians()
          : result.getBestTarget().getYaw()
          , 0);

      vx = xRateLimiterAimbot.calculate(Math.cos(forwardSpeed));
      vy = yRateLimiterAimbot.calculate(Math.sin(forwardSpeed));
      vw = wRateLimiterAimbot.calculate(rotationSpeed);

        
    // } else if (m_controller.getHID().getAButton()) { // we are still intending to search
    //   // rn this just stops the robot but we might want it to look at some target position or sm

    } else { // no target found (just drive normally)
      double maxSpeed = Constants.DriveConstants.kMaxLinearSpeed;
      vx = m_controller.getRawAxis(0) * maxSpeed;
      vy = m_controller.getRawAxis(1) * maxSpeed;
      vw = m_controller.getRawAxis(4) * 0.1;
      
      // Apply slew rate limits
      vx = xRateLimiterDriving.calculate(vx);
      vy = yRateLimiterDriving.calculate(vy);
      vw = wRateLimiterDriving.calculate(vw);
    }
    
    RobotContainer.distanceFromTarget = range;

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
