// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Underroller;
import frc.robot.subsystems.HendersonFeeder;
import frc.robot.subsystems.HendersonLauncher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final Underroller m_underroller = new Underroller();
  private final Arm m_arm = new Arm();
  private final HendersonFeeder m_feeder = new HendersonFeeder();
  private final HendersonLauncher m_launcher  = new HendersonLauncher();

  // The driver's controller
  CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  CommandXboxController m_operatorController = new CommandXboxController(OIConstants.kOperatorControllerPort);

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    configureNamedCommands();
    configureAutoChooser();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true, true),
            m_robotDrive));

      //           m_arm.setDefaultCommand(
      // new RunCommand(
      //   () -> m_arm.set(-ArmConstants.kMaxArmSpeed*
      //     MathUtil.applyDeadband(m_operatorController.getRightY(),
      //     ArmConstants.kArmDeadband)),m_arm));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    m_driverController.b().whileTrue(Commands.run(() -> m_robotDrive.setX(),m_robotDrive));

    m_operatorController.leftBumper().whileTrue(m_underroller.runUnderroller().withName("Intaking"));
    m_operatorController.rightBumper().whileTrue(m_underroller.reverseUnderroller().withName("Outtaking"));


// m_operatorController.start().onTrue(m_arm.toggleArmEnableCommand());

    m_operatorController.povDown().onTrue(m_arm.setArmGoalCommand(ArmConstants.kArmPickupAngleRads));
    m_operatorController.povUp().onTrue(m_arm.setArmGoalCommand(ArmConstants.kArmShootingAngleRads));
    m_operatorController.x().whileTrue(Commands.startEnd(m_launcher::run,m_launcher::stop,m_launcher));
    m_operatorController.b().whileTrue(Commands.startEnd(m_feeder::run,m_feeder::stop,m_feeder));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  private void configureNamedCommands() {
      NamedCommands.registerCommand("none", Commands.none());
      NamedCommands.registerCommand("waitOne", Commands.waitSeconds(1));
    }

  private void configureAutoChooser() {
    autoChooser.setDefaultOption("Nothing", Commands.none());
  }
   
  // public Command leftStageAuto() {
  //     return new PathPlannerAuto("LeftStage");
  // }

  public Command intakeNote() {
    return Commands.sequence(Commands.parallel(m_launcher.runReverse(),m_feeder.runReverse()));
  }
}
