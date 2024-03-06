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
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel;

// CONSTANTS
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.MotorContants;

public class Intake extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private CANSparkMax m_intake;
  private RelativeEncoder m_intakeEncoder;
  private SparkPIDController m_intakePIDController;

  // private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  // private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
  // private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));

  // SysIdRoutine routine;

  public Intake() {
    this.m_intake = new CANSparkMax(IntakeConstants.kIntakeCanId, MotorType.kBrushless);

    // Factory reset, so we get the SPARK MAX to a known state before configuring them. Useful in case a SPARK MAX is swapped out.
    m_intake.restoreFactoryDefaults();

    m_intakeEncoder = m_intake.getEncoder();
    
    m_intakePIDController = m_intake.getPIDController();
    m_intakePIDController.setFeedbackDevice(m_intakeEncoder);

    // m_intakePIDController.setReference(0, null, 0, 0, null)

    m_intakeEncoder.setPositionConversionFactor(IntakeConstants.kEncoderPositionFactor);
    m_intakeEncoder.setVelocityConversionFactor(IntakeConstants.kEncoderVelocityFactor);

    m_intakePIDController.setP( IntakeConstants.kP ,0);
    m_intakePIDController.setI( IntakeConstants.kI ,0);
    m_intakePIDController.setD( IntakeConstants.kD ,0);
    m_intakePIDController.setFF(IntakeConstants.kFF,0);
    // m_intakePIDController.setOutputRange(IntakeConstants.kIntakeMinOutput, IntakeConstants.kIntakeMaxOutput, 0);
    m_intakePIDController.setOutputRange(-1, 1, 0);

    // second slot
    m_intakePIDController.setP( IntakeConstants.kP ,1);
    m_intakePIDController.setI( IntakeConstants.kI ,1);
    m_intakePIDController.setD( IntakeConstants.kD ,1);
    m_intakePIDController.setFF(IntakeConstants.kFF,1);
    m_intakePIDController.setOutputRange(-1, 1, 1);

    setCoast();
    m_intake.setSmartCurrentLimit(MotorContants.kMotorCurrentLimit);

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during operation, it will maintain the above configurations.
    m_intake.burnFlash();

    m_intakeEncoder.setPosition(0);

    setCoast();

    // // Creates a SysIdRoutine
    // routine = new SysIdRoutine(
    //   new SysIdRoutine.Config(),
    //   new SysIdRoutine.Mechanism(this::voltageIntake, 
    //       log -> {
    //       log.motor("intake")
    //           .voltage(
    //               m_appliedVoltage.mut_replace(
    //                 m_intake.get() * RobotController.getBatteryVoltage(), Volts))
    //           .linearPosition(m_distance.mut_replace(m_intake.getEncoder().getPosition(), Meters))
    //           .linearVelocity(
    //               m_velocity.mut_replace(m_intake.getEncoder().getVelocity(), MetersPerSecond));
    //       },
    //   this
    // ));
  }

  // private void voltageIntake(Measure<Voltage> volts){
  //   m_intake.setVoltage(volts.in(Volts));
  // }

  /** sets intake idlemode to brake */
  public void setBrake() {
    m_intake.setIdleMode(CANSparkMax.IdleMode.kBrake);
  }

  /** sets intake idlemode to coast */
  public void setCoast() {
    m_intake.setIdleMode(CANSparkMax.IdleMode.kCoast);
  }

  public double getCurrent() {
    return m_intake.getOutputCurrent();
  }

  /** should return motor velocity in meters */
  public double getVelocity() {
    return m_intakeEncoder.getVelocity();
  }

  /** move intake motor to suck the note in */
  public void intake() {
    SmartDashboard.putString("Intake State", "intake");

    // m_intake.set(.8);
    m_intakePIDController.setReference(MotorContants.kIntakeSpeed, CANSparkMax.ControlType.kVelocity, 0);
  }

  /** move intake motor to suck the note in */
  public void intakeDistance(double distance) {
    SmartDashboard.putString("Intake State", "intake-distance");

    // m_intake.set(.8);
    m_intakePIDController.setReference(distance, CANSparkMax.ControlType.kPosition, 1);
  }

  /** stop intake motor */
  public void stop() {
    SmartDashboard.putString("Intake State", "stopped");

    // m_intake.set(0);
    m_intakePIDController.setReference(0, CANSparkMax.ControlType.kVelocity, 0);
  }
  
  // /** move intake motor to push the note out */
  // public void intakeOut() {
  //   // System.out.println("Intake out");
  //   SmartDashboard.putString("Intake State", "Out");

  //   m_intake.set(-.8);
  //   // m_intakePIDController.setReference(-MotorContants.kIntakeSpeed, CANSparkMax.ControlType.kVelocity, 0);
  // }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake Encoder Position", m_intakeEncoder.getPosition());
    SmartDashboard.putNumber("Intake Encoder Velocity", m_intakeEncoder.getVelocity());
    SmartDashboard.putNumber("Intake Temp", m_intake.getMotorTemperature());

    SmartDashboard.putNumber("Intake Current", getCurrent());
  } 
  
  // public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
  //   return routine.quasistatic(direction);
  // }

  // public Command sysIdDynamic(SysIdRoutine.Direction direction) {
  //   return routine.dynamic(direction);
  // }
}
