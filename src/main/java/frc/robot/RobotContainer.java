// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.drive.SwerveDriveSubsystem;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.superstructure.Superstructure;
import yams.mechanisms.swerve.utility.SwerveInputStream;
import yams.mechanisms.velocity.FlyWheel;
import frc.robot.subsystems.shooter.Flywheel;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


public class RobotContainer {

  final CommandXboxController driverXbox = new CommandXboxController(0);
  

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
    superstructure.setDefaultCommand(superstructure.setGoal(Superstructure.Goal.IDLE));
  }

  private void configureBindings() {
    // Zero the gyro with Start + Back — use this if the field-relative heading drifts
    driverXbox.a().onTrue(swerve.zeroGyro());

    driverXbox.b().whileTrue(superstructure.setGoal(Superstructure.Goal.SCORING));
  }

  public Command getAutonomousCommand() {
    return new PathPlannerAuto("Path1");
  }
}
