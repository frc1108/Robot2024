// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkLimitSwitch.Type;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HendersonConstants;

public class HendersonFeeder extends SubsystemBase {
  private final CANSparkMax m_leftMotor = new CANSparkMax(HendersonConstants.kLeftFeederMotorCanId,MotorType.kBrushless);
  private final CANSparkMax m_rightMotor = new CANSparkMax(HendersonConstants.kRightFeederMotorCanId,MotorType.kBrushless);
  private final RelativeEncoder m_encoder;

  /** Creates a new HendersonFeeder. */
  public HendersonFeeder() {
    m_leftMotor.restoreFactoryDefaults();
    m_rightMotor.restoreFactoryDefaults();
    m_leftMotor.setIdleMode(IdleMode.kBrake);
    m_rightMotor.setIdleMode(IdleMode.kBrake);
    m_rightMotor.follow(m_leftMotor, true);
    m_encoder = m_leftMotor.getEncoder();


    m_leftMotor.burnFlash();
    m_rightMotor.burnFlash();

  }
  public void setSpeed(double speed){
    m_leftMotor.set(speed);
  }

  public boolean getBeamBreak(){
    return m_leftMotor.getForwardLimitSwitch(Type.kNormallyOpen).isPressed();
  }

  public Command runCommand(){
    return run(()->setSpeed(0.5));
  }

  public Command runStopCommand(){
    return Commands.sequence(runOnce(()->setSpeed(0.5)),
             Commands.race(Commands.waitSeconds(2),
                           Commands.waitUntil(this::getBeamBreak)),
             runOnce(()->setSpeed(0)).withName("Beam Feeder"));
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Beam Break Pressed", getBeamBreak());
    // This method will be called once per scheduler run
  }
}