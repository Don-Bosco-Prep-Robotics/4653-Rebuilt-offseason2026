// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.drive.SwerveDriveSubsystem;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeFrame;
import frc.robot.subsystems.intake.IntakeRollers;
import frc.robot.subsystems.intake.Intake.IntakeGoal;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.Superstructure.Goal;
import yams.mechanisms.swerve.utility.SwerveInputStream;
import yams.mechanisms.velocity.FlyWheel;
import frc.robot.subsystems.shooter.Flywheel;

import static edu.wpi.first.units.Units.Degrees;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


public class RobotContainer {

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
                () -> driverXbox.getRightX())
            .withAllianceRelativeControl();


  public RobotContainer() {
    configureBindings();
    swerve.setDefaultCommand(swerve.drive(driveAngularVelocity));
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
    operatorXbox.povDown().onFalse(
      Commands.parallel(
          intake.setGoalCommand(IntakeGoal.IDLE),
          superstructure.setGoal(Goal.IDLE)
      ));
    operatorXbox.povUp().onTrue(intake.setGoalCommand(IntakeGoal.STOWED));
  }

  public Command getAutonomousCommand() {
    return new PathPlannerAuto("Path1");
  }
}
