package frc.robot.subsystems;

import frc.robot.Constants.MotorContants;
import frc.robot.Constants.ShooterConstants;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel;

// import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Shooter extends SubsystemBase {
  private CANSparkMax m_bottom;
  private CANSparkMax m_top;

  private RelativeEncoder m_bottomEncoder;
  private RelativeEncoder m_topEncoder;

  private SparkPIDController m_bottomPIDController;
  private SparkPIDController m_topPIDController;

  // private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  // private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
  // private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));

  // SysIdRoutine routine;

  public Shooter() {
    m_bottom = new CANSparkMax(ShooterConstants.kBottomCanId, MotorType.kBrushless);
    m_top = new CANSparkMax(ShooterConstants.kTopCanId, MotorType.kBrushless);

    // Factory reset, so we get the SPARKS MAX to a known state before configuring
    // them. Useful in case a SPARK MAX is swapped out.
    m_bottom.restoreFactoryDefaults();
    m_top.restoreFactoryDefaults();

    m_bottom.setInverted(true);
    m_top.setInverted(false);

    // Setup encoders and PID controllers
    m_bottomEncoder = m_bottom.getEncoder();
    m_topEncoder = m_top.getEncoder();

    m_bottomPIDController = m_bottom.getPIDController();
    m_bottomPIDController.setFeedbackDevice(m_bottomEncoder);
    m_topPIDController = m_top.getPIDController();
    m_topPIDController.setFeedbackDevice(m_topEncoder);

    m_bottomEncoder.setPositionConversionFactor(ShooterConstants.kBottomEncoderPositionFactor);
    m_bottomEncoder.setVelocityConversionFactor(ShooterConstants.kBottomEncoderVelocityFactor);
    m_topEncoder.setPositionConversionFactor(ShooterConstants.kTopEncoderPositionFactor);
    m_topEncoder.setVelocityConversionFactor(ShooterConstants.kTopEncoderVelocityFactor);

    m_bottomPIDController.setP(ShooterConstants.kBottomP  ,0);
    m_bottomPIDController.setI(ShooterConstants.kBottomI  ,0);
    m_bottomPIDController.setD(ShooterConstants.kBottomD  ,0);
    m_bottomPIDController.setFF(ShooterConstants.kBottomFF,0);
    m_bottomPIDController.setOutputRange(-1, 1,0);

    m_topPIDController.setP(ShooterConstants.kTopP  ,0);
    m_topPIDController.setI(ShooterConstants.kTopI  ,0);
    m_topPIDController.setD(ShooterConstants.kTopD  ,0);
    m_topPIDController.setFF(ShooterConstants.kTopFF,0);
    m_topPIDController.setOutputRange(-1, 1,0);
    
    m_bottomPIDController.setP(ShooterConstants.kBottomP  ,1);
    m_bottomPIDController.setI(ShooterConstants.kBottomI  ,1);
    m_bottomPIDController.setD(ShooterConstants.kBottomD  ,1);
    m_bottomPIDController.setFF(ShooterConstants.kBottomFF,1);
    m_bottomPIDController.setOutputRange(-1, 1,1);

    m_topPIDController.setP(ShooterConstants.kTopP  ,1);
    m_topPIDController.setI(ShooterConstants.kTopI  ,1);
    m_topPIDController.setD(ShooterConstants.kTopD  ,1);
    m_topPIDController.setFF(ShooterConstants.kTopFF,1);
    m_topPIDController.setOutputRange(-1, 1,1);

    setCoast();
    m_bottom.setSmartCurrentLimit(MotorContants.kMotorCurrentLimit);
    m_top.setSmartCurrentLimit(MotorContants.kMotorCurrentLimit);

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.
    m_bottom.burnFlash();
    m_top.burnFlash();

    m_bottomEncoder.setPosition(0);
    m_topEncoder.setPosition(0);

    // Creates a SysIdRoutine
    // routine = new SysIdRoutine(
    //   new SysIdRoutine.Config(),
    //   new SysIdRoutine.Mechanism(this::voltageShoot,
    //       log -> {
    //         log.motor("shoot-top")
    //           .voltage(
    //           m_appliedVoltage.mut_replace(
    //           m_top.get() * RobotController.getBatteryVoltage(), Volts))
    //             .linearPosition(m_distance.mut_replace(m_top.getEncoder().getPosition(),
    //             Meters))
    //           .linearVelocity(
    //             m_velocity.mut_replace(m_top.getEncoder().getVelocity(), MetersPerSecond));
    //         log.motor("shoot-bottom")
    //         .voltage(
    //         m_appliedVoltage.mut_replace(
    //         m_bottom.get() * RobotController.getBatteryVoltage(), Volts))
    //         .linearPosition(m_distance.mut_replace(m_bottom.getEncoder().getPosition(),
    //         Meters))
    //         .linearVelocity(
    //         m_velocity.mut_replace(m_bottom.getEncoder().getVelocity(),
    //         MetersPerSecond));
    //       },
    //       this));

    // // Creates a SysIdRoutine
    // routine = new SysIdRoutine(
    // new SysIdRoutine.Config(),
    // new SysIdRoutine.Mechanism(this::voltageShoot,
    //   log -> {
    //   log.motor("shoot-top")
    //   .voltage(
    //     m_appliedVoltage.mut_replace(m_top.get() * RobotController.getBatteryVoltage(), Volts))
    //     .linearPosition(m_distance.mut_replace(m_top.getEncoder().getPosition(), Meters))
    //     .linearVelocity(m_velocity.mut_replace(m_top.getEncoder().getVelocity(), MetersPerSecond));
    //   },
    //   this
    // ));

    // // Creates a SysIdRoutine
    // routine = new SysIdRoutine(
    // new SysIdRoutine.Config(),
    // new SysIdRoutine.Mechanism(this::voltageShoot,
    //   log -> {
    //   log.motor("shoot-bottom")
    //   .voltage(
    //   m_appliedVoltage.mut_replace(
    //   m_bottom.get() * RobotController.getBatteryVoltage(), Volts))
    //   .linearPosition(m_distance.mut_replace(m_bottom.getEncoder().getPosition(),
    //   Meters))
    //   .linearVelocity(
    //   m_velocity.mut_replace(m_bottom.getEncoder().getVelocity(),
    //   MetersPerSecond));
    //   },
    //   this
    // ));
  }

  // private void voltageShoot(Measure<Voltage> volts) {
  //   m_bottom.setVoltage(volts.in(Volts));
  //   m_top.setVoltage(volts.in(Volts));
  // }

  // Set the shooter motors to brake
  public void setBrake() {
    m_bottom.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_top.setIdleMode(CANSparkMax.IdleMode.kBrake);
  }

  // Set the shooter motors to coast
  public void setCoast() {
    m_bottom.setIdleMode(CANSparkMax.IdleMode.kCoast);
    m_top.setIdleMode(CANSparkMax.IdleMode.kCoast);
  }
  
  public double getCurrent() {
    return m_top.getOutputCurrent();
  }

  // Shoot
  public void shoot() {
    SmartDashboard.putString("Shooter State", "shoot");

    // m_bottom.set(.8);
    // m_top.set(.8);

    m_bottomPIDController.setReference(MotorContants.kShootingSpeed, CANSparkMax.ControlType.kVelocity, 0);
    m_topPIDController.setReference(MotorContants.kShootingSpeed, CANSparkMax.ControlType.kVelocity, 0);
  }

  public void intake() {
    SmartDashboard.putString("Shooter State", "intake");

    // m_bottom.set(.8);
    // m_top.set(.8);

    m_bottomPIDController.setReference(MotorContants.kIntakeSpeed, CANSparkMax.ControlType.kVelocity, 0);
    m_topPIDController.setReference(MotorContants.kIntakeSpeed, CANSparkMax.ControlType.kVelocity, 0);
  }

  public void intakeDistance(double distance) {
    SmartDashboard.putString("Shooter State", "intake-distance");

    m_bottomPIDController.setReference(distance, CANSparkMax.ControlType.kPosition, 1);
    m_topPIDController.setReference(distance, CANSparkMax.ControlType.kPosition, 1);
  }

  public void shooterIntake() {
    SmartDashboard.putString("Shooter State", "shooter-intake");

    // m_bottom.set(-.8);
    // m_top.set(-.8);

    m_bottomPIDController.setReference(-MotorContants.kIntakeSpeed, CANSparkMax.ControlType.kVelocity, 0);
    m_topPIDController.setReference(-MotorContants.kIntakeSpeed, CANSparkMax.ControlType.kVelocity, 0);
  }

  // Stop Shooters
  public void stop() {
    SmartDashboard.putString("Shooter State", "stopped");

    // m_bottom.set(0);
    // m_top.set(0);

    m_bottomPIDController.setReference(0, CANSparkMax.ControlType.kVelocity, 0);
    m_topPIDController.setReference(0, CANSparkMax.ControlType.kVelocity, 0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter Top Encoder Position", m_topEncoder.getPosition());
    SmartDashboard.putNumber("Shooter Top Encoder Velocity", m_topEncoder.getVelocity());
    SmartDashboard.putNumber("Shooter Top Temp", m_top.getMotorTemperature());
    SmartDashboard.putNumber("Shooter Top Current", m_top.getOutputCurrent());

    SmartDashboard.putNumber("Shooter Bottom Encoder Position", m_bottomEncoder.getPosition());
    SmartDashboard.putNumber("Shooter Bottom Encoder Velocity", m_bottomEncoder.getVelocity());
    SmartDashboard.putNumber("Shooter Bottom Temp", m_bottom.getMotorTemperature());
    SmartDashboard.putNumber("Shooter Bottom Current", m_bottom.getOutputCurrent());
  }

  // public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
  //   return routine.quasistatic(direction);
  // }

  // public Command sysIdDynamic(SysIdRoutine.Direction direction) {
  //   return routine.dynamic(direction);
  // }
}
