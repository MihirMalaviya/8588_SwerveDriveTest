// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;

import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.WristConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;

import com.revrobotics.CANSparkMax;

import com.revrobotics.CANSparkBase;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel;

/** A robot wrist subsystem that moves with a motion profile. */
public class Wrist extends TrapezoidProfileSubsystem {
  private final CANSparkMax m_motor;
  private final RelativeEncoder m_encoder;
  private final SparkPIDController m_PIDController; 

  private final double defaultPosition = Units.degreesToRadians(60); // radians
  private       double currentPosition = Units.degreesToRadians(60); // radians
  private final double offset = Units.degreesToRadians(0); // radians

  private final ArmFeedforward m_feedforward =
      new ArmFeedforward(
          WristConstants.kSVolts, WristConstants.kGVolts,
          WristConstants.kVVoltSecondPerRad, WristConstants.kAVoltSecondSquaredPerRad);
 
  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));

  SysIdRoutine routine;

  /** Create a new Wrist. */
  public Wrist() {
    super(new TrapezoidProfile.Constraints(WristConstants.kMaxVelocityRadPerSecond, WristConstants.kMaxAccelerationRadPerSecSquared), WristConstants.kWristOffsetRads);
    
    m_motor = new CANSparkMax(WristConstants.kArmCanId, CANSparkLowLevel.MotorType.kBrushless);

    // Factory reset, so we get the SPARK MAX to a known state before configuring them. Useful in case a SPARK MAX is swapped out.
    m_motor.restoreFactoryDefaults();

    m_encoder = m_motor.getAlternateEncoder(WristConstants.kCountsPerRev); // MIGHT NEED TO CHANGE IF WE DONT GET THROUGHBORE IN
    m_PIDController = m_motor.getPIDController();

    m_encoder.setPositionConversionFactor(WristConstants.kEncoderPositionFactor);
    m_encoder.setVelocityConversionFactor(WristConstants.kEncoderVelocityFactor);

    // m_motor.setSmartCurrentLimit(MotorContants.kMotorCurrentLimit);

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during operation, it will maintain the above configurations.
    m_motor.burnFlash();

    // m_encoder.setPosition(0);

    m_PIDController.setP(1);
    m_PIDController.setI(0);
    m_PIDController.setD(0);
    
    m_PIDController.setFeedbackDevice(m_encoder);

    setWristGoalDefaultCommand();

    // Creates a SysIdRoutine
    routine = new SysIdRoutine(
        new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism(this::voltageAim,
            log -> {
              log.motor("aim")
                  .voltage(
                      m_appliedVoltage.mut_replace(
                          m_motor.get() * RobotController.getBatteryVoltage(), Volts))
                  .linearPosition(m_distance.mut_replace(m_encoder.getPosition(), Meters))
                  .linearVelocity(
                      m_velocity.mut_replace(m_encoder.getVelocity(), MetersPerSecond));
              },
          this
        ));
  }

  private void voltageAim(Measure<Voltage> volts) {
    m_motor.setVoltage(volts.in(Volts));
  }

  @Override
  public void useState(TrapezoidProfile.State setpoint) {
    // Calculate the feedforward from the sepoint
    double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
        
    // Set the setpoint for the PID controller
    m_PIDController.setReference(setpoint.position, CANSparkBase.ControlType.kPosition, 0, feedforward / 12.0);
  }

  public Command setWristGoalCommand(double kWristOffsetRads) {
    return Commands.runOnce(() -> setGoal(kWristOffsetRads + offset), this);
  }
  
  public Command setWristGoalDefaultCommand() {
    return Commands.runOnce(() -> setGoal(defaultPosition + offset), this);
  }
  
  public Command incrementUp() {
    currentPosition += Units.degreesToRadians(5);
    return Commands.runOnce(() -> setGoal(currentPosition + offset), this);
  }
  
  public Command incrementDown() {
    currentPosition -= Units.degreesToRadians(5);
    return Commands.runOnce(() -> setGoal(currentPosition + offset), this);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Wrist Encoder Position", m_encoder.getPosition());
    SmartDashboard.putNumber("Wrist Encoder Velocity", m_encoder.getVelocity());
    SmartDashboard.putNumber("Wrist Temp", m_motor.getMotorTemperature());
  }
  
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return routine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return routine.dynamic(direction);
  }
}
