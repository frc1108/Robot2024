// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Brake extends SubsystemBase {
  /** Creates a new Brake. */
  private final CANSparkMax m_motor = new CANSparkMax(51, MotorType.kBrushed);
  public Brake() {
    m_motor.restoreFactoryDefaults();
    m_motor.setInverted(true);
    m_motor.setIdleMode(IdleMode.kBrake);
    m_motor.setSmartCurrentLimit(30);
    m_motor.burnFlash();
  }

  public void set(double speed) {
    m_motor.set(speed);
  }

  public Command brake() {
    return Commands.startEnd(()->set(0.75),()->set(0),this).withTimeout(0.3);
  }

  public Command unbrake() {
    return Commands.startEnd(()->set(-0.75),()->set(0),this).withTimeout(0.3);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
