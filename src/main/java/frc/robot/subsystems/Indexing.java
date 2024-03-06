package frc.robot.subsystems;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel;

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
// CONSTANTS
// import frc.robot.Constants.IntakeConstants; 
import frc.robot.Constants.MotorContants;
import frc.robot.Constants.IndexingConstants;

// NOTE we will move arm to a seperate subsystem once sysid is done

public class Indexing extends SubsystemBase {
  private boolean loaded = false;

  private CANSparkMax m_left;
  private CANSparkMax m_right;

  private RelativeEncoder m_leftEncoder;
  private RelativeEncoder m_rightEncoder;

  private SparkPIDController m_leftPIDController;
  private SparkPIDController m_rightPIDController;

  // private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  // private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
  // private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));

  // SysIdRoutine routine;

  public Indexing() {
    m_left = new CANSparkMax(IndexingConstants.kLeftCanId, MotorType.kBrushless);
    m_right = new CANSparkMax(IndexingConstants.kRightCanId, MotorType.kBrushless);
    
    // Factory reset, so we get the SPARKS MAX to a known state before configuring them. Useful in case a SPARK MAX is swapped out.
    m_left.restoreFactoryDefaults();
    m_right.restoreFactoryDefaults();

    m_left.setInverted(true);
    m_right.setInverted(false);

    // Setup encoders and PID controllers
    m_leftEncoder = m_left.getEncoder();
    m_rightEncoder = m_right.getEncoder();

    m_leftPIDController = m_left.getPIDController();
    m_leftPIDController.setFeedbackDevice(m_leftEncoder);
    m_rightPIDController = m_right.getPIDController();
    m_rightPIDController.setFeedbackDevice(m_rightEncoder);

    m_leftEncoder.setPositionConversionFactor(IndexingConstants.kLeftEncoderPositionFactor);
    m_leftEncoder.setVelocityConversionFactor(IndexingConstants.kLeftEncoderVelocityFactor);
    m_rightEncoder.setPositionConversionFactor(IndexingConstants.kRightEncoderPositionFactor);
    m_rightEncoder.setVelocityConversionFactor(IndexingConstants.kRightEncoderVelocityFactor);

    m_leftPIDController.setP(IndexingConstants.kP  ,0);
    m_leftPIDController.setI(IndexingConstants.kI  ,0);
    m_leftPIDController.setD(IndexingConstants.kD  ,0);
    m_leftPIDController.setFF(IndexingConstants.kFF,0);
    m_leftPIDController.setOutputRange(-1, 1,0);
    
    m_rightPIDController.setP(IndexingConstants.kP  ,0);
    m_rightPIDController.setI(IndexingConstants.kI  ,0);
    m_rightPIDController.setD(IndexingConstants.kD  ,0);
    m_rightPIDController.setFF(IndexingConstants.kFF,0);
    m_rightPIDController.setOutputRange(-1, 1,0);
    
    m_leftPIDController.setP(IndexingConstants.kP  ,1);
    m_leftPIDController.setI(IndexingConstants.kI  ,1);
    m_leftPIDController.setD(IndexingConstants.kD  ,1);
    m_leftPIDController.setFF(IndexingConstants.kFF,1);
    m_leftPIDController.setOutputRange(-1, 1,1);
    
    m_rightPIDController.setP(IndexingConstants.kP  ,1);
    m_rightPIDController.setI(IndexingConstants.kI  ,1);
    m_rightPIDController.setD(IndexingConstants.kD  ,1);
    m_rightPIDController.setFF(IndexingConstants.kFF,1);
    m_rightPIDController.setOutputRange(-1, 1,1);

    setCoast();
    m_left.setSmartCurrentLimit(MotorContants.kMotorCurrentLimit);
    m_right.setSmartCurrentLimit(MotorContants.kMotorCurrentLimit);

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during operation, it will maintain the above configurations.
    m_left.burnFlash();
    m_right.burnFlash();

    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);   
    
    
    // Additional initialization stuff here if needed
    setCoast();
    
    // // Creates a SysIdRoutine
    // routine = new SysIdRoutine(
    //   new SysIdRoutine.Config(),
    //   new SysIdRoutine.Mechanism(this::voltageIndexing, 
    //       log -> {
    //       log.motor("indexing")
    //           .voltage(
    //               m_appliedVoltage.mut_replace(
    //                 m_left.get() * RobotController.getBatteryVoltage(), Volts))
    //           .linearPosition(m_distance.mut_replace(m_left.getEncoder().getPosition(), Meters))
    //           .linearVelocity(
    //               m_velocity.mut_replace(m_left.getEncoder().getVelocity(), MetersPerSecond));
    //       },
    //   this
    // ));
  }

  // private void voltageIndexing(Measure<Voltage> volts){
  //   m_left.setVoltage(volts.in(Volts));
  //   m_right.setVoltage(-volts.in(Volts));
  // }

  public double getCurrent() {
    return m_right.getOutputCurrent();
  }

  public double getVelocity() {
    return m_rightEncoder.getVelocity();
  }

  public void setBrake() {
    m_left.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_right.setIdleMode(CANSparkMax.IdleMode.kBrake);
  }

  public void setCoast() {
    m_left.setIdleMode(CANSparkMax.IdleMode.kCoast);
    m_right.setIdleMode(CANSparkMax.IdleMode.kCoast);
  }

  public void intake() {
    SmartDashboard.putString("Indexing State", "intake");

    // m_left.set(.8);
    // m_right.set(.8);

    m_leftPIDController.setReference(MotorContants.kIntakeSpeed, CANSparkMax.ControlType.kVelocity, 0);
    m_rightPIDController.setReference(MotorContants.kIntakeSpeed, CANSparkMax.ControlType.kVelocity, 0);
  }
  
  public void intakeDistance(double distance) {
    SmartDashboard.putString("Intake State", "intake-distance");

    m_leftPIDController.setReference(distance, CANSparkMax.ControlType.kPosition, 1);
    m_rightPIDController.setReference(distance, CANSparkMax.ControlType.kPosition, 1);
  }

  /** move index motors to push the note out */
  public void shooterIntake() {
    SmartDashboard.putString("Indexing State", "shooter-intake");

    // m_left.set(-.8);
    // m_right.set(-.8);
    
    m_leftPIDController.setReference(-MotorContants.kIntakeSpeed, CANSparkMax.ControlType.kVelocity, 0);
    m_rightPIDController.setReference(-MotorContants.kIntakeSpeed, CANSparkMax.ControlType.kVelocity, 0);
  }

  /** move index motors to push the note out */
  public void shoot() {
    SmartDashboard.putString("Indexing State", "shoot");

    // m_left.set(.8);
    // m_right.set(.8);
    
    m_leftPIDController.setReference(MotorContants.kShootingSpeed, CANSparkMax.ControlType.kVelocity, 0);
    m_rightPIDController.setReference(MotorContants.kShootingSpeed, CANSparkMax.ControlType.kVelocity, 0);
  }

  public void stop() {
    SmartDashboard.putString("Indexing State", "stopped");
    
    // m_right.set(0);
    // m_left.set(0);

    m_leftPIDController.setReference(0, CANSparkMax.ControlType.kVelocity, 0);
    m_rightPIDController.setReference(0, CANSparkMax.ControlType.kVelocity, 0);
  }

  public void setLoaded(boolean b) {
    loaded = b;
    SmartDashboard.putString("State", loaded ? "loaded" : "empty");
    // System.out.println(loaded ? "loaded" : "empty");
    //System.out.println(loaded ? "loaded" : "empty");
  }

  public boolean isLoaded() {
    SmartDashboard.putString("State", loaded ? "loaded" : "empty");
    // System.out.println(loaded ? "loaded" : "empty");
    //System.out.println(loaded ? "loaded" : "empty");
    return loaded;
  }

  public void periodic() {
    SmartDashboard.putNumber("Indexing Left Encoder Position", m_leftEncoder.getPosition());
    SmartDashboard.putNumber("Indexing Left Encoder Velocity", m_leftEncoder.getVelocity());
    SmartDashboard.putNumber("Indexing Left Temp", m_left.getMotorTemperature());

    SmartDashboard.putNumber("Indexing Right Encoder Position", m_rightEncoder.getPosition());
    SmartDashboard.putNumber("Indexing Right Encoder Velocity", m_rightEncoder.getVelocity());
    SmartDashboard.putNumber("Indexing Right Temp", m_right.getMotorTemperature());

    SmartDashboard.putNumber("Indexing Current", getCurrent());
  }

  // public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
  //   return routine.quasistatic(direction);
  // }

  // public Command sysIdDynamic(SysIdRoutine.Direction direction) {
  //   return routine.dynamic(direction);
  // }
}