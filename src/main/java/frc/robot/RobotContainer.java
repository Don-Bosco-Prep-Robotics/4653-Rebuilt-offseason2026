// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.drive.SwerveDriveSubsystem;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeGoal;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.Superstructure.Goal;
import yams.mechanisms.swerve.utility.SwerveInputStream;
import frc.robot.subsystems.shooter.Flywheel;

import static edu.wpi.first.units.Units.Degrees;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;


public class RobotContainer {

  private final SendableChooser<Command> autoChooser;

  final CommandXboxController driverXbox = new CommandXboxController(0);
  final CommandXboxController operatorXbox = new CommandXboxController(1);

  private final SwerveDriveSubsystem swerve = new SwerveDriveSubsystem();
  private final Intake intake = new Intake();
  private final Indexer indexer = new Indexer();
  // private final Superstructure superstructure = new Superstructure(
  //   new FlyWheel(ShooterConstants.flywheelLeftConfig, ShooterConstants.flywheelLeftSmartSpark),
  //   new FlyWheel(ShooterConstants.flywheelRightConfig, ShooterConstants.flywheelRightSmartSpark),
  //   intake,
  //   indexer);
  private final Superstructure superstructure = new Superstructure(
    new Flywheel(ShooterConstants.flywheelLeftMotorConfig, ShooterConstants.flywheelLeftConfig, ShooterConstants.flywheelLeftMotor, "LeftFlywheel"),
    new Flywheel(ShooterConstants.flywheelRightMotorConfig, ShooterConstants.flywheelRightConfig, ShooterConstants.flywheelRightMotor, "RightFlywheel"),
    intake,
    indexer);
  // private final Flywheel flywheelLeft = new Flywheel(ShooterConstants.flywheelLeftMotorConfig, ShooterConstants.flywheelLeftConfig, ShooterConstants.flywheelLeftMotor, "LeftFlywheel");
  // private final Flywheel flywheelRight = new Flywheel(ShooterConstants.flywheelRightMotorConfig, ShooterConstants.flywheelRightConfig, ShooterConstants.flywheelRightMotor, "RightFlywheel");

  private final SwerveInputStream driveAngularVelocity =
      swerve.getAngularVelocityStream(
                driverXbox::getLeftY,
                driverXbox::getLeftX,
                () -> driverXbox.getRightX());
            // .withAllianceRelativeControl();

  private final Trigger intakeDown;
  private final Trigger intakeStow;

  public RobotContainer() {

    //Annoyingly, there's no way to run setState(SHOOTING), wait(4), setState(IDLE) because shooting is a continuous command. This works though.
    NamedCommands.registerCommand(
        "Auto Shoot",
        superstructure.setGoal(Goal.SCORING)
            .withTimeout(5)
            .andThen(superstructure.setGoal(Goal.IDLE))
    );
    NamedCommands.registerCommand(
        "Auto Intake", 
        Commands.sequence(
          intake.setGoalCommand(IntakeGoal.INTAKING).withTimeout(5),
          intake.setGoalCommand(IntakeGoal.STOWED).withTimeout(2)
        )
    );

    intakeDown = new EventTrigger("Intake Down Trigger").whileTrue(
      Commands.sequence(
        intake.setGoalCommand(IntakeGoal.INTAKING)
      )
    );
    intakeStow = new EventTrigger("Intake Stow Trigger").whileTrue(
      Commands.sequence(
        intake.setGoalCommand(IntakeGoal.STOWED)
      )
    );

    configureBindings();
    swerve.setDefaultCommand(swerve.driveFieldRelative(driveAngularVelocity));
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    // superstructure.setDefaultCommand(superstructure.setGoal(Superstructure.Goal.IDLE));
  }

  private void configureBindings() {
    // Zero the gyro with Start + Back — use this if the field-relative heading drifts
    driverXbox.a().onTrue(swerve.zeroGyro());

    operatorXbox.b().onTrue(superstructure.setGoal(Superstructure.Goal.SCORING));
    operatorXbox.b().onFalse(superstructure.setGoal(Superstructure.Goal.IDLE));
    operatorXbox.y().onTrue(intake.setFrameEncoder(Degrees.of(0)));

    operatorXbox.povDown().onTrue(intake.setGoalCommand(IntakeGoal.INTAKING));
    operatorXbox.povDown().and(operatorXbox.leftTrigger()).onTrue(
      Commands.parallel(
        intake.setGoalCommand(IntakeGoal.OUTTAKING),
        superstructure.setGoal(Superstructure.Goal.OUTTAKING)
      ));
    operatorXbox.povDown().or(operatorXbox.leftTrigger()).onFalse(
      Commands.parallel(
          intake.setGoalCommand(IntakeGoal.IDLE),
          superstructure.setGoal(Goal.IDLE)
      ));
    operatorXbox.povUp().onTrue(intake.setGoalCommand(IntakeGoal.STOWED));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
