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
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
  m_rightMotor.follow(m_leftMotor, true);

  m_encoder = m_leftMotor.getEncoder();
  m_pid = m_leftMotor.getPIDController();
  m_pid.setFeedbackDevice(m_encoder);

  m_encoder.setPositionConversionFactor(ArmConstants.kArmEncoderPositionFactor);
  m_encoder.setVelocityConversionFactor(ArmConstants.kArmEncoderVelocityFactor);
  m_encoder.setPosition(ArmConstants.kArmOffsetRads);

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
  public void periodic(){
    super.setGoal(m_goal);
    super.periodic();

    SmartDashboard.putNumber("Arm Goal", m_goal);
    SmartDashboard.putNumber("Arm Position", getPositionRadians());
    SmartDashboard.putNumber("Motor Output", getMotorCurrent());
    SmartDashboard.putBoolean("Left Follower", m_leftMotor.isFollower());
    SmartDashboard.putBoolean("Right Follower", m_rightMotor.isFollower());
    SmartDashboard.putNumber("Left Output", m_leftMotor.get());
    SmartDashboard.putNumber("Right Output", m_rightMotor.get());
    SmartDashboard.putNumber("Position Conv", m_encoder.getPositionConversionFactor());
    SmartDashboard.putNumber("Vel Conv", m_encoder.getVelocityConversionFactor());
  }

  @Override
  public void useState(TrapezoidProfile.State setpoint) {
  // Calculate the feedforward from the sepoint
  double feedforward = m_feedforward.calculate(setpoint.position,
                                               setpoint.velocity);

  SmartDashboard.putNumber("Feedforward", feedforward);
  SmartDashboard.putNumber("Setpoint Position", setpoint.position);
  SmartDashboard.putNumber("Setpoint Velocity", setpoint.velocity);

  
  // Add the feedforward to the PID output to get the motor output
  m_pid.setReference(setpoint.position, // - ArmConstants.kArmOffsetRads,
                     ControlType.kPosition, 0, feedforward);
}

public Command setArmGoalCommand(double goal) {
return Commands.runOnce(() -> setArmGoal(goal), this);
}

public void set(double speed) {
  m_leftMotor.set(speed);
}

public double getPositionRadians() {
  return m_encoder.getPosition(); // + ArmConstants.kArmOffsetRads;
}

public Command setArmManual(DoubleSupplier speed) {
  return Commands.run(()->setArmGoal(getArmGoal()+speed.getAsDouble()/(2*Math.PI)),this);
}

public double getArmGoal() {
  return m_goal;
}

public void setArmGoal(double goal) {
  
  m_goal = MathUtil.clamp(goal,ArmConstants.kArmOffsetRads-0.1,ArmConstants.kArmMaxRads+0.1);
}

public void setEncoderPosition(double position) {
  m_encoder.setPosition(position);
}

public double getMotorCurrent(){
  return m_leftMotor.getOutputCurrent();
}

// public Command toggleArmEnableCommand(){
//   return  runOnce(()->{
//     if(m_enabled) {
//       disable();
//       m_enabled = false;
//     } else {
//       enable();
//       m_enabled = true;
//     }
//   });
// }
}
  
