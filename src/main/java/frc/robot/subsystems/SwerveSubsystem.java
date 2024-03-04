// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DriveConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.kauailabs.navx.frc.AHRS;


public class SwerveSubsystem extends SubsystemBase {

  SwerveModule m_frontLeftModule = new SwerveModule(
    DriveConstants.kFrontLeftTurningID, DriveConstants.kFrontLeftDrivingID, 
    DriveConstants.kFrontLeftAngularOffset, DriveConstants.kFrontLeftLocation);

  SwerveModule m_frontRightModule = new SwerveModule(
    DriveConstants.kFrontRightTurningID, DriveConstants.kFrontRightDrivingID, 
    DriveConstants.kFrontRightAngularOffset, DriveConstants.kFrontRightLocation);

  SwerveModule m_backRightModule = new SwerveModule(
    DriveConstants.kBackRightTurningID, DriveConstants.kBackRightDrivingID, 
    DriveConstants.kBackRightAngularOffset, DriveConstants.kBackRightLocation);

  SwerveModule m_backLeftModule = new SwerveModule(
    DriveConstants.kBackLeftTurningID, DriveConstants.kBackLeftDrivingID, 
    DriveConstants.kBackLeftAngularOffset, DriveConstants.kBackLeftLocation);

  SwerveDriveKinematics m_DriveKinematics = new SwerveDriveKinematics(
      m_frontLeftModule.m_moduleLocation, m_frontRightModule.m_moduleLocation,
      m_backRightModule.m_moduleLocation, m_backLeftModule.m_moduleLocation);

  public AHRS ahrs = new AHRS();

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
    m_DriveKinematics,
    Rotation2d.fromDegrees(ahrs.getAngle()),
    new SwerveModulePosition[] {
        m_frontLeftModule.getPosition(),
        m_frontRightModule.getPosition(),
        m_backLeftModule.getPosition(),
        m_backRightModule.getPosition()
    });

  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem() {
      
  }

  public double getYaw() {
    return ahrs.getYaw();
  }

  public void zeroYaw() {
    ahrs.zeroYaw();
  }

  public void drive(ChassisSpeeds chassisSpeeds) {

    SwerveModuleState[] states = m_DriveKinematics.toSwerveModuleStates(chassisSpeeds);

    // Normalize these speeds if they become impossibly fast
    SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.kMaxLinearSpeed);

    m_frontLeftModule.setDesiredState(states[0]);
    m_frontRightModule.setDesiredState(states[1]);
    m_backRightModule.setDesiredState(states[2]);
    m_backLeftModule.setDesiredState(states[3]);
    
    // ChassisSpeeds adjSpeeds = new ChassisSpeeds()
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
      Rotation2d.fromDegrees(ahrs.getAngle()),
      new SwerveModulePosition[] {
          m_frontLeftModule.getPosition(),
          m_frontRightModule.getPosition(),
          m_backLeftModule.getPosition(),
          m_backRightModule.getPosition()
      },
      pose);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Update the odometry
    m_odometry.update(
      Rotation2d.fromDegrees(ahrs.getAngle()),
      new SwerveModulePosition[] {
          m_frontLeftModule.getPosition(),
          m_frontRightModule.getPosition(),
          m_backLeftModule.getPosition(),
          m_backRightModule.getPosition()
      });

      SmartDashboard.putNumber("Pose X", getPose().getX());
      SmartDashboard.putNumber("Pose Y", getPose().getY());
      SmartDashboard.putNumber("Pose Rotation", getPose().getRotation().getDegrees());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
