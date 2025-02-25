// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class DriveConstants {
    public static final int kFrontLeftTurningID = 10;
    public static final int kFrontLeftDrivingID = 11;
    
    public static final int kFrontRightTurningID = 14;
    public static final int kFrontRightDrivingID = 15;
    
    public static final int kBackRightTurningID = 16;
    public static final int kBackRightDrivingID = 17;

    public static final int kBackLeftTurningID = 12;
    public static final int kBackLeftDrivingID = 13;

    // MODULE PIDs
    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;

    public static final double kDrivingP = 0.055;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / ((5676 / 60) * (0.0762 * Math.PI) / 4.71);

    // BOT PIDs
    public static final double kRotationP = 5.0;
    public static final double kRotationI = 0.0;
    public static final double kRotationD = 0.5;

    public static final double kDriveP = 10.0;
    public static final double kDriveI = 0.0;
    public static final double kDriveD = 0.25;

    public static final double kDriveReduction = 4.71;
    public static final double kAngleReduction = 46.42;

    public static final int kDrivingMaxCurrent = 60;
    public static final int kTurningMaxCurrent = 30;

    public static final double kMaxLinearSpeed = 4.80; // m/sec

    public static final double kWheelOffset = 12.5;

    public static final Translation2d kFrontLeftLocation = new Translation2d(kWheelOffset, kWheelOffset);
    public static final Translation2d kFrontRightLocation = new Translation2d(kWheelOffset, -kWheelOffset);
    public static final Translation2d kBackRightLocation = new Translation2d(-kWheelOffset, -kWheelOffset);
    public static final Translation2d kBackLeftLocation = new Translation2d(-kWheelOffset, kWheelOffset);
  
    public static final double kFrontLeftAngularOffset = Math.PI / 2;
    public static final double kFrontRightAngularOffset = Math.PI;
    public static final double kBackRightAngularOffset = 3 * Math.PI / 2;
    public static final double kBackLeftAngularOffset = 0;

    public static final double kAnglePositionConversionFactor = 2 * Math.PI; // (0 - 1) to (0 - 2pi)
    public static final double kAngleVelocityConversionFactor = (2 * Math.PI) / 60.0;

    public static final double kDrivePositionConversionFactor = 0.05082576649756735; // yall know where this comes from
    public static final double kDriveVelocityConversionFactor = kDrivePositionConversionFactor / 60;

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kAnglePositionConversionFactor; // radians

    /* EXPLAINING SLEW RATE LIMITING
     * The value of the slew rate limiter is the number of units to change per second. For example,
     * a value of 2.0 means that it takes 0.5 seconds (1/2) to get from 0 to 1.
     */
    public static final double kXYSlewRate = 4.0;
    public static final double kRotationalSlewRate = 2.0;

    // PID constants should be tuned per robot
    public static final double kLinearP = 0.5;
    public static final double kLinearI = 0;
    public static final double kLinearD = 0.1;

    public static final double kAngularP = 10.03;
    public static final double kAngularI = 0;
    public static final double kAngularD = 0.003;
  }

  public static final class VisionConstants {
    public static final String CAMERA_NAME = "photonvision";

    // Constants about how your camera is mounted to the robot
    public static final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(15); // Angle "up" from horizontal
    public static final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(24); // Height above floor
  
    // we need to put the right values once we mount camera
    public static final Transform3d ROBOT_TO_CAM = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0.0, 0.0, 0.0)); 
  }

  /** Constants for the Intake Subsystem */
  public static final class IntakeConstants {
    // CAN IDs
    public static final int kIntakeCanId = 20;

    // MEASUREMENTS !!!!!!!!! NOT FINAL
    public static final double kWheelDiameter = Units.inchesToMeters(4.0); // meters

    // UNIT CONVERSION
    public static final double kEncoderPositionFactor = kWheelDiameter * Math.PI; // meters
    public static final double kEncoderVelocityFactor = (kWheelDiameter * Math.PI) / 60.0; // meters per second

    // PID tuning
    public static final double kP  = 0.01;
    public static final double kI  = 0.0;
    public static final double kD  = 0.019;
    public static final double kFF = 0.0;

    public static final double kMaxVel = 1000.0 * kEncoderPositionFactor;
    public static final double kMaxAcc = 1000.0;
  }

  /** Constants for the Indexing Subsystem */
  public static final class IndexingConstants {
    // CAN IDs
    public static final int kLeftCanId = 21;
    public static final int kRightCanId = 22;

    // MEASUREMENTS
    public static final double kWheelDiameter = Units.inchesToMeters(3.0); // meters
    public static final double kGearRatio = 1.0 / 16.0; // 16:1 gear ratio

    // UNIT CONVERSION
    public static final double kLeftEncoderPositionFactor =  kWheelDiameter * Math.PI * kGearRatio; // meters
    public static final double kLeftEncoderVelocityFactor = (kWheelDiameter * Math.PI * kGearRatio) / 60.0; // meters per second

    public static final double kRightEncoderPositionFactor = kLeftEncoderPositionFactor; // meters
    public static final double kRightEncoderVelocityFactor = kLeftEncoderVelocityFactor; // meters per second

    // PID tuning
    public static final double kP  = 1;
    public static final double kI  = 0;
    public static final double kD  = 0;
    public static final double kFF = 0;
  }

  /** Constants for the Shooter Subsystem */
  public static final class ShooterConstants {
    public static final int kBottomCanId = 24;
    public static final int kTopCanId = 25;

    // MEASUREMENTS
    public static final double kBottomWheelDiameter = Units.inchesToMeters(3.0); // meters
    public static final double kTopWheelDiameter    = Units.inchesToMeters(4.0); // meters

    // UNIT CONVERSION
    public static final double kBottomEncoderPositionFactor = kBottomWheelDiameter * Math.PI; // meters
    public static final double kBottomEncoderVelocityFactor = (kBottomWheelDiameter * Math.PI) / 60.0; // meters per second

    public static final double kTopEncoderPositionFactor = kTopWheelDiameter * Math.PI; // meters
    public static final double kTopEncoderVelocityFactor = (kTopWheelDiameter * Math.PI) / 60.0; // meters per second

    // PID tuning
    public static final double kBottomP  = 1;
    public static final double kBottomI  = 0;
    public static final double kBottomD  = 0;
    public static final double kBottomFF = 0;

    public static final double kTopP  = 1;
    public static final double kTopI  = 0;
    public static final double kTopD  = 0;
    public static final double kTopFF = 0;
  }

  /** Constants for the Wrist Constants */
  public static final class WristConstants {
    // CAN IDs
    public static final int kArmCanId = 23;

    // THROUGHBORE ENCODER
    public static final int kCountsPerRev = 8192;

    // UNIT CONVERSION
    public static final double kEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second
    
    // // PID tuning
    // public static final double kP = 1;
    // public static final double kI = 0;
    // public static final double kD = 0;
    // public static final double kFF = 0;

    // These are fake gains; in actuality these must be determined individually for each robot
    public static final double kSVolts = 1;
    public static final double kGVolts = 1;
    public static final double kVVoltSecondPerRad = 0.5;
    public static final double kAVoltSecondSquaredPerRad = 0.1;

    public static final double kMaxVelocityRadPerSecond = 3;
    public static final double kMaxAccelerationRadPerSecSquared = 1; // TODO
    
    // The offset of the arm from the horizontal in its neutral position,
    // measured from the horizontal
    public static final double kWristOffsetRads = 0.5;
  }

  public static final class MotorContants {
    public static final double kArmSpeed      = 0.5; // meters per second
    public static final double kIntakeSpeed   = 100.0; // meters per second
    public static final double kShootingSpeed = 10.0; // meters per second
    
    public static final double kIntakeDistance   = 10.0; // meters
    public static final double kIndexingDistance = 0.5; // meters

    public static final int kMotorCurrentLimit = 50; // amps
  }
}
