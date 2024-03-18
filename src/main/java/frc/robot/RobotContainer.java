// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.ArmConstants;
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
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.PathPlannerLogging;

import monologue.Monologue;
import monologue.Annotations.Log;
import monologue.Logged;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer implements Logged{
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final Underroller m_underroller = new Underroller();
  private final Arm m_arm = new Arm();
  private final HendersonFeeder m_feeder = new HendersonFeeder();
  private final HendersonLauncher m_launcher  = new HendersonLauncher();
  private final LEDSubsystem m_led = new LEDSubsystem();

  // The driver's controller
  CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  CommandXboxController m_operatorController = new CommandXboxController(OIConstants.kOperatorControllerPort);

  @Log.NT private final Field2d m_field;
  @Log.NT private final SendableChooser<Command> m_autoChooser;
  private int m_invertDriveAlliance = -1;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() { 
    m_field = new Field2d();
    
    // Robot configs
    configureButtonBindings();
    configureNamedCommands();
    m_autoChooser = AutoBuilder.buildAutoChooser();
    setupMonologue();
    setupPathPlannerLog();
    // Default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                m_invertDriveAlliance*MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                m_invertDriveAlliance*MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
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
    m_operatorController.a().whileTrue(Commands.runEnd(()->m_feeder.set(1),()->m_feeder.set(0),m_feeder));//Commands.startEnd(m_feeder::run,m_feeder::stop,m_feeder));
    m_operatorController.b().whileTrue(Commands.runEnd(()->m_feeder.set(-0.55),()->m_feeder.set(0),m_feeder));//Commands.startEnd(m_feeder::run,m_feeder::stop,m_feeder));
    m_operatorController.back().onTrue(intakeNote()); //whileTrue(Commands.runEnd(()->m_feeder.set(-0.55),()->m_feeder.set(0),m_feeder));//Commands.startEnd(m_feeder::run,m_feeder::stop,m_feeder));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }

  private void configureNamedCommands() {
      // NamedCommands.registerCommand("none", Commands.none());
      NamedCommands.registerCommand("LaunchNote", shoot());
      NamedCommands.registerCommand("IntakeNote", intakeNote());
      NamedCommands.registerCommand("ShootBackwards", shootBackwards());
      NamedCommands.registerCommand("CenteringNote", centeringNote());
      // NamedCommands.registerCommand("waitOne", Commands.waitSeconds(1));
    }

  private void configureAutoChooser() {
  //   autoChooser.setDefaultOption("Nothing", Commands.none());
  //   autoChooser.addOption("Test Auto", TestAuto());
  //   autoChooser.addOption("Two Note Left", TwoNoteLeft());
  //   autoChooser.addOption("Two Note Center", TwoNoteCenter());
  //   autoChooser.addOption("Three Note Center", ThreeNoteCenter());
  //   SmartDashboard.putData("Auto Chooser",autoChooser);
   }

  public void configureWithAlliance(Alliance alliance) {
    m_led.startCrowdMeter(alliance);
    m_invertDriveAlliance = (alliance == Alliance.Blue)?-1:1;
  }
   
  // public Command TestAuto() {
  //     return new PathPlannerAuto("Test Auto");
  // }

  // public Command TwoNoteLeft() {
  //     return new PathPlannerAuto("Two Note Left");
  // }

  // public Command TwoNoteCenter() {
  //     return new PathPlannerAuto("Two Note Center");
  // }

  // public Command ThreeNoteCenter() {
  //     return new PathPlannerAuto("Three Note Center");
  // }

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

    public Command shootBackwards() {
    return Commands.sequence(
      Commands.runOnce(()->m_feeder.disableLimitSwitches()),
      m_arm.setArmGoalCommand(ArmConstants.kArmPickupAngleRads-Units.degreesToRadians(2)),
      Commands.waitSeconds(0.2),
      Commands.runOnce(()->m_feeder.set(1)),
      Commands.waitSeconds(0.25),
      Commands.runOnce(()->m_launcher.set(0.8)),
      Commands.waitSeconds(0.75)).andThen(
        Commands.parallel(
                        Commands.runOnce(()->m_launcher.set(0.0)),
                        Commands.runOnce(()->m_feeder.set(0.0)))
                          );
  }

  public Command centeringNote() {
    return Commands.sequence(
      Commands.runOnce(()->m_feeder.disableLimitSwitches()),
      Commands.parallel(Commands.runOnce(()->m_launcher.set(-0.1)),
                        Commands.runOnce(()->m_feeder.set(-0.5))),
      Commands.waitSeconds(0.1)).andThen(
        Commands.parallel(
                        Commands.runOnce(()->m_launcher.set(0.0)),
                        Commands.runOnce(()->m_feeder.set(0.0)))
                          );
  }

  public void setupMonologue() {
    boolean fileOnly = false;
    boolean lazyLogging = false;
    Monologue.setupMonologue(this,"RobotContainer",fileOnly,lazyLogging);
  }

  public void runPeriodic() {
    Monologue.setFileOnly(DriverStation.isFMSAttached());
    Monologue.updateAll();
  }

  private void setupPathPlannerLog() {
    // Logging callback for current robot pose
    PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
        // Do whatever you want with the pose here
        m_field.setRobotPose(pose);
    });

    // Logging callback for target robot pose
    PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
      // Do whatever you want with the pose here
      m_field.getObject("target pose").setPose(pose);
    });

    // Logging callback for the active path, this is sent as a list of poses
    PathPlannerLogging.setLogActivePathCallback((poses) -> {
      // Do whatever you want with the poses here
      m_field.getObject("path").setPoses(poses);
    });
  }
}
