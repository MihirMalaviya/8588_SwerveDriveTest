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
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.List;

import com.kauailabs.navx.frc.AHRS;

public class SwerveSubsystem extends SubsystemBase {
  
  private final StructArrayPublisher<SwerveModuleState> publisher;

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

  SwerveModuleState[] states = m_DriveKinematics.toSwerveModuleStates(new ChassisSpeeds());

  List<SwerveModule> modules = List.of(this.m_frontLeftModule, this.m_frontRightModule, this.m_backLeftModule, this.m_backRightModule);

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

  private double oldTurningP  = m_frontLeftModule.getTurningPidController().getP();
  private double oldTurningI  = m_frontLeftModule.getTurningPidController().getI();
  private double oldTurningD  = m_frontLeftModule.getTurningPidController().getD();
  private double oldTurningFF = m_frontLeftModule.getTurningPidController().getFF();

  private double oldDrivingP  = m_frontLeftModule.getDrivingPidController().getP();
  private double oldDrivingI  = m_frontLeftModule.getDrivingPidController().getI();
  private double oldDrivingD  = m_frontLeftModule.getDrivingPidController().getD();
  private double oldDrivingFF = m_frontLeftModule.getDrivingPidController().getFF();

  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem() {
    // Start publishing an array of module states with the "/SwerveStates" key
    publisher = NetworkTableInstance.getDefault()
      .getStructArrayTopic("/SwerveStates", SwerveModuleState.struct).publish();
    
    SmartDashboard.putNumber("Turning P",  oldTurningP);
    SmartDashboard.putNumber("Turning I",  oldTurningI);
    SmartDashboard.putNumber("Turning D",  oldTurningD); 
    SmartDashboard.putNumber("Turning FF", oldTurningFF);
    
    SmartDashboard.putNumber("Driving P",  oldDrivingP);
    SmartDashboard.putNumber("Driving I",  oldDrivingI);
    SmartDashboard.putNumber("Driving D",  oldDrivingD); 
    SmartDashboard.putNumber("Driving FF", oldDrivingFF);
  }

  public double getYaw() {
    return ahrs.getYaw();
  }

  public void zeroYaw() {
    ahrs.zeroYaw();
  }

  public void drive(ChassisSpeeds chassisSpeeds) {

    states = m_DriveKinematics.toSwerveModuleStates(chassisSpeeds);

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

  private void updatePids() {
    
    double currentTurningP  = SmartDashboard.getNumber("Turning P",  oldTurningP);
    double currentTurningI  = SmartDashboard.getNumber("Turning I",  oldTurningI);
    double currentTurningD  = SmartDashboard.getNumber("Turning D",  oldTurningD); 
    double currentTurningFF = SmartDashboard.getNumber("Turning FF", oldTurningFF);

    double currentDrivingP  = SmartDashboard.getNumber("Driving P",  oldDrivingP);
    double currentDrivingI  = SmartDashboard.getNumber("Driving I",  oldDrivingI);
    double currentDrivingD  = SmartDashboard.getNumber("Driving D",  oldDrivingD); 
    double currentDrivingFF = SmartDashboard.getNumber("Driving FF", oldDrivingFF);

    if (
      currentTurningP  != oldTurningP ||
      currentTurningI  != oldTurningI ||
      currentTurningD  != oldTurningD ||
      currentTurningFF != oldTurningFF
    ) {
      modules.forEach(m -> {
        m.getTurningPidController().setP( currentTurningP);
        m.getTurningPidController().setI( currentTurningI);
        m.getTurningPidController().setD( currentTurningD);
        m.getTurningPidController().setFF(currentTurningFF);
      });
      // System.out.println("updated");
    }

    if (
      currentDrivingP  != oldDrivingP ||
      currentDrivingI  != oldDrivingI ||
      currentDrivingD  != oldDrivingD ||
      currentDrivingFF != oldDrivingFF
    ) {
      modules.forEach(m -> {
        m.getDrivingPidController().setP( currentDrivingP);
        m.getDrivingPidController().setI( currentDrivingI);
        m.getDrivingPidController().setD( currentDrivingD);
        m.getDrivingPidController().setFF(currentDrivingFF);
      });
      // System.out.println("updated");
    }
    
    oldTurningP  = currentTurningP;
    oldTurningI  = currentTurningI;
    oldTurningD  = currentTurningD;
    oldTurningFF = currentTurningFF;
    
    oldDrivingP  = currentDrivingP;
    oldDrivingI  = currentDrivingI;
    oldDrivingD  = currentDrivingD;
    oldDrivingFF = currentDrivingFF;
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Periodically send a set of module states
    publisher.set(states);

    // Update the odometry
    m_odometry.update(
      Rotation2d.fromDegrees(ahrs.getAngle()),
      new SwerveModulePosition[] {
          m_frontLeftModule.getPosition(),
          m_frontRightModule.getPosition(),
          m_backLeftModule.getPosition(),
          m_backRightModule.getPosition()
      });
      
      updatePids();

      SmartDashboard.putNumber("Pose X", getPose().getX());
      SmartDashboard.putNumber("Pose Y", getPose().getY());
      SmartDashboard.putNumber("Pose Rotation", getPose().getRotation().getDegrees());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
