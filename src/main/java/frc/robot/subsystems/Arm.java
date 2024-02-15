// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import frc.robot.Constants.ArmConstants;

public class Arm extends TrapezoidProfileSubsystem {
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
  m_leftMotor.setInverted(false);
  m_rightMotor.setInverted(true);

  m_rightMotor.follow(m_leftMotor);

  m_encoder = m_leftMotor.getEncoder();
  m_encoder.setPosition(ArmConstants.kArmOffsetRads);
  m_pid = m_leftMotor.getPIDController();
  m_pid.setFeedbackDevice(m_encoder);

  m_encoder.setPositionConversionFactor(ArmConstants.kArmEncoderPositionFactor);
  m_encoder.setVelocityConversionFactor(ArmConstants.kArmEncoderVelocityFactor);

  m_pid.setP(ArmConstants.kP);
m_pid.setI(ArmConstants.kI);
m_pid.setD(ArmConstants.kD);
m_pid.setFF(ArmConstants.kFF);
m_pid.setOutputRange(ArmConstants.kMinOutput,
        ArmConstants.kMaxOutput);

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
  protected void useState(TrapezoidProfile.State state) {
    // Use the computed profile state here.
  }
}
