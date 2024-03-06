// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

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

  // // Odometry class for tracking robot pose
  // SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
  //   m_DriveKinematics,
  //   Rotation2d.fromDegrees(ahrs.getAngle()),
  //   new SwerveModulePosition[] {
  //       m_frontLeftModule.getPosition(),
  //       m_frontRightModule.getPosition(),
  //       m_backLeftModule.getPosition(),
  //       m_backRightModule.getPosition()
  //   });

  PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(), PoseStrategy.CLOSEST_TO_REFERENCE_POSE, RobotContainer.m_camera, VisionConstants.ROBOT_TO_CAM);

  /* Here we use SwerveDrivePoseEstimator so that we can fuse odometry readings. The numbers used
  below are robot specific, and should be tuned. */
  private final SwerveDrivePoseEstimator m_poseEstimator;

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

    m_poseEstimator =
        new SwerveDrivePoseEstimator(
            m_DriveKinematics,
            ahrs.getRotation2d(),
            new SwerveModulePosition[] {
              m_frontLeftModule.getPosition(),
              m_frontRightModule.getPosition(),
              m_backLeftModule.getPosition(),
              m_backRightModule.getPosition()
            },
            new Pose2d(),
            // these are std devs, they may need to be tuned accordingly for our bot
            VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
            VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));
            
    // // need to change this to where the robot is on the atual field somehow
    // photonPoseEstimator.setLastPose(getPose());

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
    return m_poseEstimator.getEstimatedPosition();
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    return photonPoseEstimator.update();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_poseEstimator.resetPosition(
      ahrs.getRotation2d(),
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
      // currentTurningI  != oldTurningI ||
      currentTurningD  != oldTurningD ||
      currentTurningFF != oldTurningFF
    ) {
      modules.forEach(m -> {
        m.getTurningPidController().setP( currentTurningP);
        // m.getTurningPidController().setI( currentTurningI);
        m.getTurningPidController().setD( currentTurningD);
        m.getTurningPidController().setFF(currentTurningFF);
      });
      // System.out.println("updated");
    }

    if (
      currentDrivingP  != oldDrivingP ||
      // currentDrivingI  != oldDrivingI ||
      currentDrivingD  != oldDrivingD ||
      currentDrivingFF != oldDrivingFF
    ) {
      modules.forEach(m -> {
        m.getDrivingPidController().setP( currentDrivingP);
        // m.getDrivingPidController().setI( currentDrivingI);
        m.getDrivingPidController().setD( currentDrivingD);
        m.getDrivingPidController().setFF(currentDrivingFF);
      });
      // System.out.println("updated");
    }
    
    oldTurningP  = currentTurningP;
    // oldTurningI  = currentTurningI;
    oldTurningD  = currentTurningD;
    oldTurningFF = currentTurningFF;
    
    oldDrivingP  = currentDrivingP;
    // oldDrivingI  = currentDrivingI;
    oldDrivingD  = currentDrivingD;
    oldDrivingFF = currentDrivingFF;
  }
  
  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_poseEstimator.update(
        ahrs.getRotation2d(), // might need to be changed
        new SwerveModulePosition[] {
          m_frontLeftModule.getPosition(),
          m_frontRightModule.getPosition(),
          m_backLeftModule.getPosition(),
          m_backRightModule.getPosition()
        });

    // apply vision measurements
    // this must be calculated based either on latency or timestamps.
    Optional<EstimatedRobotPose> estimatedGlobalPose = getEstimatedGlobalPose(m_poseEstimator.getEstimatedPosition());

    if (estimatedGlobalPose.isPresent()) {
      m_poseEstimator.addVisionMeasurement(
          getEstimatedGlobalPose(m_poseEstimator.getEstimatedPosition()).get().estimatedPose.toPose2d(),
          Timer.getFPGATimestamp() - 0.3);
      
      SmartDashboard.putBoolean("Vision Pose Estimate", true);
    } else {
      SmartDashboard.putBoolean("Vision Pose Estimate", false);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Periodically send a set of module states
    publisher.set(states);
      
    updateOdometry();

    updatePids();

    Pose2d pose = getPose();
    SmartDashboard.putNumber("Pose X", pose.getX());
    SmartDashboard.putNumber("Pose Y", pose.getY());
    SmartDashboard.putNumber("Pose Rotation", pose.getRotation().getDegrees());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
