// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import frc.robot.Constants.ArmConstants;
import monologue.Logged;
import monologue.Annotations.Log;

public class Arm extends TrapezoidProfileSubsystem implements Logged{
  /** Creates a new Arm. */
  private final CANSparkMax m_leftMotor = new CANSparkMax(ArmConstants.kLeftArmMotorCanId, MotorType.kBrushless);
  private final CANSparkMax m_rightMotor = new CANSparkMax(ArmConstants.kRightArmMotorCanId, MotorType.kBrushless);

  private final SparkPIDController m_pid;
  private final RelativeEncoder m_encoder;

  private final ArmFeedforward m_feedforward = 
  new ArmFeedforward(
    ArmConstants.kSVolts, ArmConstants.kGVolts, 
    ArmConstants.kVVoltSecondPerRad, ArmConstants.kAVoltSecondSquaredPerRad);
  private double m_goal = ArmConstants.kArmOffsetRads;

  private boolean m_climbEnabled = false;
  
  public Arm() {
    super(
        // The constraints for the generated profiles
        new TrapezoidProfile.Constraints(
          ArmConstants.kMaxVelocityRadPerSecond, 
          ArmConstants.kMaxAccelerationRadPerSecSquared),
        // The initial position of the mechanism
        ArmConstants.kArmOffsetRads);

  m_leftMotor.restoreFactoryDefaults();
  m_rightMotor.restoreFactoryDefaults();
  m_rightMotor.follow(m_leftMotor, true);

  m_encoder = m_leftMotor.getEncoder();
  m_pid = m_leftMotor.getPIDController();
  m_pid.setFeedbackDevice(m_encoder);

  m_encoder.setPositionConversionFactor(ArmConstants.kArmEncoderPositionFactor);
  m_encoder.setVelocityConversionFactor(ArmConstants.kArmEncoderVelocityFactor);
  m_encoder.setPosition(ArmConstants.kArmOffsetRads);

  m_pid.setP(ArmConstants.kP,ArmConstants.kSlotDefault);
  m_pid.setI(ArmConstants.kI,ArmConstants.kSlotDefault);
  m_pid.setD(ArmConstants.kD,ArmConstants.kSlotDefault);
  m_pid.setFF(ArmConstants.kFF,ArmConstants.kSlotDefault);
  m_pid.setOutputRange(ArmConstants.kMinOutput,
        ArmConstants.kMaxOutput,ArmConstants.kSlotDefault);

  // Gain schedule for climbing
  m_pid.setP(ArmConstants.kPClimb,ArmConstants.kSlotClimb);
  m_pid.setI(ArmConstants.kIClimb,ArmConstants.kSlotClimb);
  m_pid.setD(ArmConstants.kDClimb,ArmConstants.kSlotClimb);
  m_pid.setFF(ArmConstants.kFFClimb,ArmConstants.kSlotClimb);
  m_pid.setOutputRange(ArmConstants.kMinOutputClimb,
        ArmConstants.kMaxOutputClimb,ArmConstants.kSlotClimb);

  // Apply current limit and idle mode
  m_leftMotor.setIdleMode(IdleMode.kBrake);
  m_rightMotor.setIdleMode(IdleMode.kBrake);
  m_leftMotor.setSmartCurrentLimit(ArmConstants.kArmMotorCurrentLimit);
  m_rightMotor.setSmartCurrentLimit(ArmConstants.kArmMotorCurrentLimit);

  // Save the SPARK MAX configurations.
  m_leftMotor.burnFlash();
  m_rightMotor.burnFlash();
  }

  @Override
  public void periodic(){
    super.setGoal(m_goal);
    super.periodic();
  }

  @Override
  public void useState(TrapezoidProfile.State setpoint) {
  // Calculate the feedforward from the sepoint
  double feedforward = m_feedforward.calculate(setpoint.position,
                                               setpoint.velocity);

  this.log("Feedforward", feedforward);
  this.log("Setpoint Position", setpoint.position);
  this.log("Setpoint Velocity", setpoint.velocity);

  
  // Add the feedforward to the PID output to get the motor output
    if (!m_climbEnabled) {
      m_pid.setReference(setpoint.position, // - ArmConstants.kArmOffsetRads,
                      ControlType.kPosition, ArmConstants.kSlotDefault, feedforward);
    } else {
      m_pid.setReference(setpoint.position, // - ArmConstants.kArmOffsetRads,
                    ControlType.kPosition, ArmConstants.kSlotClimb, feedforward);
    }
  }

  public Command setArmGoalCommand(double goal) {
  return Commands.runOnce(() -> setArmGoal(goal), this);
  }

  public void set(double speed) {
    m_leftMotor.set(speed);
  }

  @Log.NT(key = "Arm Pos Rads")
  public double getPositionRadians() {
    return m_encoder.getPosition();
  }

  @Log.NT(key = "Arm Goal Rads")
  public double getArmGoal() {
    return m_goal;
  }

  public void setArmGoal(double goal) {
    
    m_goal = MathUtil.clamp(goal,ArmConstants.kArmOffsetRads-0.1,ArmConstants.kArmMaxRads+0.1);
  }

  public Command setArmManual(DoubleSupplier speed) {
    return Commands.run(()->setArmGoal(getArmGoal()+speed.getAsDouble()/(2*Math.PI)),this);
  }

  public void setEncoderPosition(double position) {
    m_encoder.setPosition(position);
  }

  @Log.NT(key = "Arm, Amps")
  public double getMotorCurrent(){
    return m_leftMotor.getOutputCurrent();
  }

  public void setIdle(IdleMode mode) {
    m_leftMotor.setIdleMode(mode);
  }

  public void enableClimb() {
    m_climbEnabled = true;
  }

  public void disableClimb() {
    m_climbEnabled = true;
  }
}
  
