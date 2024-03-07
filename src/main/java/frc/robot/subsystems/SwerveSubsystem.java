// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathPlannerTrajectory.State;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

public class SwerveSubsystem extends SubsystemBase {
  
  public static Translation2d blueSpeakerPos = new Translation2d(0.076, 5.55);
  public static Translation2d redSpeakerPos = new Translation2d(16.47, 5.55);
  public static Pose2d blueAmp = new Pose2d(1.82, 7.7, new Rotation2d());
  public static Pose2d redAmp = new Pose2d(14.73, 7.7, new Rotation2d());
  
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
  
  // path planner init
  public PathPlannerPath path;
  public PathPlannerTrajectory autoTraj;
  public Translation2d ajustedV = new Translation2d();
  public double rotCorrection = 0;
  
  // path PID loops
  public PIDController m_xPosPidController = new PIDController(DriveConstants.kDriveP, DriveConstants.kDriveI, DriveConstants.kDriveD);
  public PIDController m_yPosPidController = new PIDController(DriveConstants.kDriveP, DriveConstants.kDriveI, DriveConstants.kDriveD);
  public PIDController m_rotPidController = new PIDController(DriveConstants.kRotationP, DriveConstants.kRotationI, DriveConstants.kRotationD);
  
  // path planning vars
  public double trajStartTime;
  public State goalState = new State();
  public double trajElapsedTime;
  public Trajectory.State trajState;
  public Rotation2d goalHeading = new Rotation2d();
  public ChassisSpeeds adjustedSpeeds;
  public PhotonCamera camera = RobotContainer.m_camera;
  AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
  public double lastPhotonUpdateTime = 0;

  public AHRS gyro = new AHRS();

  // // Odometry class for tracking robot pose
  // SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
  //   m_DriveKinematics,
  //   Rotation2d.fromDegrees(gyro.getAngle()),
  //   new SwerveModulePosition[] {
  //       m_frontLeftModule.getPosition(),
  //       m_frontRightModule.getPosition(),
  //       m_backLeftModule.getPosition(),
  //       m_backRightModule.getPosition()
  //   });

  PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, RobotContainer.m_camera, VisionConstants.ROBOT_TO_CAM);

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
            gyro.getRotation2d(),
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
    
    // pathFollowingInit
    goalState = new State();

    // ---
    
    oldTurningP  = m_frontLeftModule.getTurningPidController().getP();
    oldTurningI  = m_frontLeftModule.getTurningPidController().getI();
    oldTurningD  = m_frontLeftModule.getTurningPidController().getD();
    oldTurningFF = m_frontLeftModule.getTurningPidController().getFF();
    
    oldDrivingP  = m_frontLeftModule.getDrivingPidController().getP();
    oldDrivingI  = m_frontLeftModule.getDrivingPidController().getI();
    oldDrivingD  = m_frontLeftModule.getDrivingPidController().getD();
    oldDrivingFF = m_frontLeftModule.getDrivingPidController().getFF();

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
    return gyro.getYaw();
  }

  public void zeroYaw() {
    gyro.zeroYaw();
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

  // load a path from file
  public void initPath(String pathName, boolean flipToRed) {

    path = PathPlannerPath.fromPathFile(pathName);

    if (flipToRed) {
      path = path.flipPath();
    }

    // geneate trajectory from path
    autoTraj = path.getTrajectory(new ChassisSpeeds(), path.getPreviewStartingHolonomicPose().getRotation());

    m_xPosPidController.reset();
    m_yPosPidController.reset();
    m_rotPidController.reset();

    goalState = autoTraj.sample(0);
    goalHeading = goalState.targetHolonomicRotation;

    // if (Robot.isSimulation() 
    // // || thisRobot.autoController.autoRoutine == autoRoutines._XTESTINGACCURACY
    // ) {
    resetOdometry(goalState.getTargetHolonomicPose());
    // }

    trajStartTime = Timer.getFPGATimestamp();
  }

  // apply control to follow each step of path
  public void followPath() {
    // get current sample
    trajElapsedTime = Timer.getFPGATimestamp() - trajStartTime;
    goalState = autoTraj.sample(trajElapsedTime);
    goalHeading = goalState.targetHolonomicRotation;

    // update xy PID controllers
    double xCorrection = m_xPosPidController.calculate(getPose().getX(), goalState.getTargetHolonomicPose().getX());
    double yCorrection = m_yPosPidController.calculate(getPose().getY(), goalState.getTargetHolonomicPose().getY());

    goalHeading = new Rotation2d(Math.toRadians(goalHeading.getDegrees()));

    // add the path velocity with the PID velocity
    Translation2d trajV = new Translation2d(goalState.velocityMps, goalHeading);
    ajustedV = new Translation2d(xCorrection, yCorrection);
    ajustedV = ajustedV.plus(trajV);

    double rot = m_rotPidController.calculate(m_poseEstimator.getEstimatedPosition().getRotation().getRadians(),
        goalHeading.getRadians());

    drive(ChassisSpeeds.fromFieldRelativeSpeeds(
      ajustedV.getX(), ajustedV.getY(), rot, m_poseEstimator.getEstimatedPosition().getRotation()));
  }

  public Command followPathCommand(String pathName) {
    PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

    return new FollowPathHolonomic(
            path,
            this::getPose, // Robot pose supplier
            this::getSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::drive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(DriveConstants.kDriveP, 0.0, DriveConstants.kDriveD), // Translation PID constants
                    new PIDConstants(DriveConstants.kRotationP, 0.0, DriveConstants.kRotationD), // Rotation PID constants
                    4.5, // Max module speed, in m/s
                    0.45, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () -> {
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            this // Reference to this subsystem to set requirements
    );
  }
  
  public boolean isPathOver() {
    double trajElapsedTime = Timer.getFPGATimestamp() - trajStartTime;
    return trajElapsedTime > autoTraj.getTotalTimeSeconds();
  }

  public void setOdomToPathInit() {
    Pose2d initPose = goalState.getTargetHolonomicPose();
    resetOdometry(initPose);
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
  
  public ChassisSpeeds getSpeeds() {
    return m_DriveKinematics.toChassisSpeeds(states);
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_poseEstimator.resetPosition(
      gyro.getRotation2d(),
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
        gyro.getRotation2d(), // might need to be changed
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
          estimatedGlobalPose.get().timestampSeconds);
          // Timer.getFPGATimestamp() - 0.3);
      
      SmartDashboard.putBoolean("Vision Pose Estimate", true);
    } else {
      SmartDashboard.putBoolean("Vision Pose Estimate", false);
    }
  }

  //set goal heading to aim at a point on the field, we are always looking away from the goal
  public void aimAtPoint(Translation2d point) {
    Translation2d currentPos = getPose().getTranslation();
    Translation2d goalPos = currentPos.minus(point);
    goalHeading = goalPos.getAngle();
  }
  
  public void goTowardPoint(Pose2d point) {
    double xP = m_xPosPidController.calculate(getPose().getX(), point.getX());
    double yP = m_yPosPidController.calculate(getPose().getY(), point.getY());

    driveClosedLoopHeading(new Translation2d(xP, yP));
  }
  
  //used in tele to just update goal heading to face the goals
  public void setGoalHeadingToGoal() {
    if (RobotContainer.isRedAlliance) {
      aimAtPoint(redSpeakerPos);
    } else {
      aimAtPoint(blueSpeakerPos);
    }
  }

  // drive with closed loop heading control while updateing goal heading
  public void driveClosedLoopHeading(Translation2d translation) {

    double rot = m_rotPidController.calculate(m_poseEstimator.getEstimatedPosition().getRotation().getRadians(),
        goalHeading.getRadians());

        
    drive(ChassisSpeeds.fromFieldRelativeSpeeds(
      translation.getX(), translation.getY(), rot, m_poseEstimator.getEstimatedPosition().getRotation()));
  }

  //used in auto when not moving to point the robot at the goal
  public void aimAtGoal() {
    setGoalHeadingToGoal();
    driveClosedLoopHeading(new Translation2d());
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
