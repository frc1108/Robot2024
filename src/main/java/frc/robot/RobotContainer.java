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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.HendersonConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.UnderrollerConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LEDSubsystem;
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
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.PathPlannerLogging;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final Field2d field;

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

    private final LEDSubsystem m_led = new LEDSubsystem();


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

            field = new Field2d();
        SmartDashboard.putData("Field", field);

        // Logging callback for current robot pose
        PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
            // Do whatever you want with the pose here
            field.setRobotPose(pose);
        });

        // Logging callback for target robot pose
        PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
          // Do whatever you want with the pose here
          field.getObject("target pose").setPose(pose);
      });

      // Logging callback for the active path, this is sent as a list of poses
      PathPlannerLogging.setLogActivePathCallback((poses) -> {
          // Do whatever you want with the poses here
          field.getObject("path").setPoses(poses);
      });

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

    //m_driverController.a().onTrue(Commands.runOnce(() -> m_robotDrive.zeroHeading()));
    m_driverController.b().whileTrue(Commands.run(() -> m_robotDrive.setX(),m_robotDrive));
    m_driverController.povLeft().onTrue(Commands.runOnce(m_led::nextPattern,m_led));


    m_operatorController.leftBumper().whileTrue(m_underroller.runUnderroller().withName("Intaking"));
    m_operatorController.rightBumper().whileTrue(m_underroller.reverseUnderroller().withName("Outtaking"));


// m_operatorController.start().onTrue(m_arm.toggleArmEnableCommand());

    m_operatorController.povDown().onTrue(m_arm.setArmGoalCommand(ArmConstants.kArmPickupAngleRads));
    m_operatorController.povUp().onTrue(m_arm.setArmGoalCommand(ArmConstants.kArmShootingAngleRads));
    m_operatorController.povRight().onTrue(m_arm.setArmGoalCommand(ArmConstants.kArmFarShootingAngleRads));
    m_operatorController.povLeft().onTrue(m_arm.setArmGoalCommand(ArmConstants.kArmDownRads));

    m_operatorController.x().whileTrue(Commands.runEnd(()->m_launcher.set(0.75),()->m_launcher.set(0),m_launcher));//Commands.startEnd(m_launcher::run,m_launcher::stop,m_launcher));
    m_operatorController.y().whileTrue(Commands.runEnd(()->m_launcher.set(-0.75),()->m_launcher.set(0),m_launcher));//Commands.startEnd(m_launcher::run,m_launcher::stop,m_launcher));
    m_operatorController.start().onTrue(shoot());
    //m_operatorController.b().whileTrue(Commands.startEnd(m_feeder::run,m_feeder::stop,m_feeder));
    m_operatorController.a().whileTrue(Commands.runEnd(()->m_feeder.set(0.55),()->m_feeder.set(0),m_feeder));//Commands.startEnd(m_feeder::run,m_feeder::stop,m_feeder));
    m_operatorController.b().whileTrue(Commands.runEnd(()->m_feeder.set(-0.55),()->m_feeder.set(0),m_feeder));//Commands.startEnd(m_feeder::run,m_feeder::stop,m_feeder));
    m_operatorController.back().onTrue(intakeNote()); //whileTrue(Commands.runEnd(()->m_feeder.set(-0.55),()->m_feeder.set(0),m_feeder));//Commands.startEnd(m_feeder::run,m_feeder::stop,m_feeder));
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
      // NamedCommands.registerCommand("none", Commands.none());
      // NamedCommands.registerCommand("waitOne", Commands.waitSeconds(1));
    }

  private void configureAutoChooser() {
    autoChooser.setDefaultOption("Nothing", Commands.none());
    autoChooser.addOption("Test Auto", TestAuto());
    SmartDashboard.putData("Auto Chooser",autoChooser);
  }

  public void configureWithAlliance(Alliance allianceColor) {

  }
   
  public Command TestAuto() {
      return new PathPlannerAuto("Test Auto");
  }

  public Command intakeNote() {
    return 
      Commands.sequence(
        m_arm.setArmGoalCommand(ArmConstants.kArmPickupAngleRads),
        Commands.waitSeconds(0.1),
        Commands.parallel(
          Commands.runOnce(()->m_launcher.set(HendersonConstants.kIntakeLauncherSpeed),m_launcher),
          Commands.runOnce(()->m_feeder.set(HendersonConstants.kIntakeFeederSpeed),m_feeder),
          Commands.runOnce(()->m_underroller.setUnderrollerspeed(UnderrollerConstants.kUnderrollerIntakeSpeed),m_underroller),
          Commands.runOnce(()->m_led.yellow()),
          Commands.runOnce(()->m_feeder.enableLimitSwitches())),
        Commands.race(Commands.waitUntil(()->m_feeder.getBeamBreak()),Commands.waitSeconds(5)),
          Commands.parallel(
          Commands.runOnce(()->m_launcher.set(0),m_launcher),
          Commands.runOnce(()->m_feeder.set(0),m_feeder),
          Commands.runOnce(()->m_underroller.setUnderrollerspeed(0),m_underroller),
          Commands.runOnce(()->m_feeder.disableLimitSwitches())),
        Commands.waitSeconds(0.5),
        Commands.runOnce(()->m_led.off())
        );
  }

  public Command shoot() {
    return Commands.sequence(
        m_arm.setArmGoalCommand(ArmConstants.kArmShootingAngleRads),
        Commands.waitSeconds(0.5),
        Commands.runOnce(()->m_launcher.set(-0.8)),
        Commands.waitSeconds(0.75),
        Commands.runOnce(()->m_feeder.set(-0.6)),
        Commands.waitSeconds(0.2),
        Commands.runOnce(()->m_feeder.set(0)),
        Commands.runOnce(()->m_launcher.set(0))
     );
  }
}
