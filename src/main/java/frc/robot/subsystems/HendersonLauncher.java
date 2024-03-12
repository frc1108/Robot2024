// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HendersonConstants;

public class HendersonLauncher extends SubsystemBase {
  private final CANSparkFlex m_leftMotor = new CANSparkFlex(HendersonConstants.kLeftLauncherMotorCanId,MotorType.kBrushless);
  private final CANSparkFlex m_rightMotor = new CANSparkFlex(HendersonConstants.kRightLauncherMotorCanId,MotorType.kBrushless);
  private final RelativeEncoder m_encoder;
  private final SparkPIDController m_pidController;

  /** Creates a new HendersonFeeder. */
  public HendersonLauncher() {
    m_leftMotor.restoreFactoryDefaults();
    m_rightMotor.restoreFactoryDefaults();
    m_leftMotor.setIdleMode(IdleMode.kCoast);
    m_rightMotor.setIdleMode(IdleMode.kCoast);
    m_rightMotor.follow(m_leftMotor, true);
    m_encoder = m_leftMotor.getEncoder();
    m_pidController = m_leftMotor.getPIDController();

    


    m_leftMotor.burnFlash();
    m_rightMotor.burnFlash();
  }

  @Override
  public void periodic() {
     SmartDashboard.putNumber("Launcher Speed",m_encoder.getVelocity());

    // This method will be called once per scheduler run
  }

  public void set(double speed) {
    m_leftMotor.set(speed);
  }

  public Command run() {
    return Commands.runOnce(()->set(0.5));
  }

  public Command runReverse() {
    return Commands.runOnce(()->set(-0.35));
  }

  public Command stop() {
    return Commands.runOnce(()->set(0));
  }
}
