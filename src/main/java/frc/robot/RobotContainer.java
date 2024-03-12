// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AutoShootAndFulcrum;
import frc.robot.commands.FeedCenterCmd;
import frc.robot.commands.FeedHoldCmd;
import frc.robot.commands.FulcrumAimCmd;
import frc.robot.commands.FulcrumCmd;
import frc.robot.commands.FulcrumTuningCmd;
import frc.robot.commands.IntakeCmd;
import frc.robot.commands.IntakeCmd2;
import frc.robot.commands.ShootAutoRpmCmd;
import frc.robot.commands.ShootCmd;
import frc.robot.commands.TurnToTargetCmd;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Fulcrum;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Leds;
import frc.utils.Position;
import frc.utils.RobotStatus;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final Intake intake = new Intake();

  private final Fulcrum fulcrum = new Fulcrum();
  private final Launcher launcher = new Launcher();
  public static final Feeder feeder = new Feeder();
  public static final Leds leds = new Leds();
  private final Climb climb = new Climb();

  // The driver's controller
  // XboxController m_driverController = new
  // XboxController(OIConstants.kDriverControllerPort);
  // The driver's controller
  CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  CommandXboxController operatorController = new CommandXboxController(OIConstants.kOperatorControllerPort);

  private final SendableChooser<Command> autoChoice;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    NamedCommands.registerCommand("ShootCmd", new ShootCmd(launcher, feeder).withTimeout(1));
    NamedCommands.registerCommand("IntakeCmd", new IntakeCmd(intake, feeder));
    NamedCommands.registerCommand("IntakeCmd2", new IntakeCmd(intake, feeder));
    NamedCommands.registerCommand("AutoAim", new TurnToTargetCmd(m_robotDrive));
    NamedCommands.registerCommand("AutoFulcrum", new FulcrumAimCmd(fulcrum));
    NamedCommands.registerCommand("FeedHold", new RunCommand(() -> feeder.stopAll(), feeder));
    NamedCommands.registerCommand("AutoShoot", new AutoShootAndFulcrum(fulcrum, launcher, feeder));

    SmartDashboard.putData("Turn To Target", new TurnToTargetCmd(m_robotDrive));
    SmartDashboard.putData("Auto Fulcrum", new FulcrumAimCmd(fulcrum));
    SmartDashboard.putData("Set Fulcrum SetPoint", new FulcrumTuningCmd(fulcrum));

    autoChoice = AutoBuilder.buildAutoChooser();

    Shuffleboard.getTab("Autonomous").add(autoChoice);
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband((m_driverController.getRightTriggerAxis()
                    - m_driverController.getLeftTriggerAxis()), OIConstants.kDriveDeadband),
                true, true),
            m_robotDrive));

    leds.setDefaultCommand(new RunCommand(() -> leds.ledState(), leds));
    
    // fulcrum.setDefaultCommand(new RunCommand(() -> fulcrum.stopFulcrum(), fulcrum));

    fulcrum.setDefaultCommand(new RunCommand(() -> fulcrum.manualFulcrum(operatorController.getRightY()*0.0), fulcrum));
  
    feeder.setDefaultCommand(new RunCommand(()-> feeder.stopAll(), feeder));
    climb.setDefaultCommand(new RunCommand(()-> climb.manualAll(0, 0), climb));
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
    
   //While Right bumper is held robot is robot relative 
    
    m_driverController.a().whileTrue(
            new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband((m_driverController.getRightTriggerAxis()
                    - m_driverController.getLeftTriggerAxis()), OIConstants.kDriveDeadband),
                false, true),
                m_robotDrive)
                );
    m_driverController.a().onTrue(new InstantCommand(()-> leds.setRobotStatus(RobotStatus.ROBOT_CENTRIC), leds));
    m_driverController.a().onFalse(new InstantCommand(()-> leds.setRobotStatus(leds.getPrevRobotStatus()), leds));




    

    m_driverController.start().onTrue(new InstantCommand(()->m_robotDrive.zeroHeading()));
    m_driverController.back().onTrue(new InstantCommand(()->m_robotDrive.restOdomWithCamData()));
    m_driverController.leftBumper().whileTrue(
        new RunCommand(()->climb.manualAll(-MathUtil.applyDeadband(m_driverController.getRightY(), OIConstants.kDriveDeadband),-MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband)), climb));
    m_driverController.leftBumper().onTrue(new InstantCommand(()-> leds.setRobotStatus(RobotStatus.CLIMB), leds));
    m_driverController.leftBumper().onFalse(new InstantCommand(()-> leds.setRobotStatus(leds.getPrevRobotStatus()), leds));
    m_driverController.rightBumper().whileTrue(
      new RunCommand(
        () -> m_robotDrive.autoAimAndDrive(
            -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
            -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband)),
        m_robotDrive)
    );
    m_driverController.b().whileTrue(
      new RunCommand(
        () -> m_robotDrive.turnToAmpAndDrive(
          -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
          -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband)), 
        m_robotDrive)
    );
    // operatorController.a().whileTrue(new RunCommand(() -> intake.intake(), intake))
    //     .onFalse(new RunCommand(() -> intake.intakeStop(), intake));
    // operatorController.b().whileTrue(new RunCommand(() -> intake.outtake(), intake))
    //     .onFalse(new RunCommand(() -> intake.intakeStop(), intake));
    // // operatorController.x().whileTrue(new RunCommand(() -> fulcrum.manualFulcrum(.5), fulcrum));
    // operatorController.x().whileTrue(new RunCommand(() -> feeder.feed(), feeder))
    //     .onFalse(new RunCommand(()-> feeder.stopAll(), feeder));
    
    // operatorController.y().whileTrue(new RunCommand(()-> launcher.lancherMaxSpeed(), launcher))
    //     .onFalse(new RunCommand(()-> launcher.stopAll(), launcher));
    operatorController.a().whileTrue(new FulcrumCmd
    (Position.INTAKE, fulcrum, false).alongWith(new IntakeCmd2(intake, feeder,fulcrum)));
    //operatorController.b().onTrue(new ShootCmd(launcher, feeder));
    operatorController.b().onTrue(new RunCommand(()->launcher.lancherMaxSpeed(), launcher ));
    operatorController.b().onTrue(new InstantCommand(()-> leds.setRobotStatus(RobotStatus.LAUNCH), leds));
    operatorController.x().onTrue(new RunCommand(() -> launcher.stopAll(), launcher)).onTrue(new RunCommand(() -> feeder.stopAll(), feeder)).onTrue(new RunCommand(() -> intake.intakeStop(), intake));
    operatorController.x().onTrue(new InstantCommand(()-> leds.setRobotStatus(RobotStatus.DEFAULT), leds));
    operatorController.y().whileTrue(new RunCommand(()-> intake.outtake(), intake));
    operatorController.rightBumper().whileTrue(new RunCommand(()-> feeder.feed(), feeder));
    operatorController.leftBumper().whileTrue(new RunCommand(()-> feeder.reversefeed(), feeder));

    operatorController.start().whileTrue(new FeedCenterCmd(feeder));

    operatorController.back().whileTrue(new RunCommand(() -> fulcrum.autoAim(), fulcrum)).whileTrue(new RunCommand(() -> launcher.launcherAutoSpeed(), launcher)).onTrue(new InstantCommand(()-> leds.setRobotStatus(RobotStatus.TARGET_LOCK), leds)).onFalse(new InstantCommand(()-> leds.setRobotStatus(leds.getPrevRobotStatus()), leds)).onFalse(new RunCommand(() -> launcher.resumeLauncher(), launcher));

    operatorController.pov(0).onTrue(new FulcrumCmd(Position.AMP, fulcrum, false));
    operatorController.pov(270).onTrue(new FulcrumCmd(Position.STOW, fulcrum, false));
    operatorController.pov(90).onTrue(new FulcrumCmd(Position.SPEAKER, fulcrum, false));
    operatorController.pov(180).onTrue(new FulcrumCmd(Position.INTAKE, fulcrum, false));
    

    // new JoystickButton(operatorController, GamePadButtons.Start)
    // .whileTrue(new InstantCommand(driveTrain::resetAll, driveTrain));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChoice.getSelected();
  }
}
