// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.drive.SwerveDriveSubsystem;
import frc.robot.subsystems.shooter.YAMSShooter;
import yams.mechanisms.swerve.utility.SwerveInputStream;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


public class RobotContainer {

  final CommandXboxController driverXbox = new CommandXboxController(0);
  

  private final SwerveDriveSubsystem swerve = new SwerveDriveSubsystem();
  private final YAMSShooter shooter = new YAMSShooter();

  private final SwerveInputStream driveAngularVelocity =
      swerve.getAngularVelocityStream(
                driverXbox::getLeftY,
                driverXbox::getLeftX,
                () -> driverXbox.getRightX())
            .withAllianceRelativeControl();


  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    swerve.setDefaultCommand(swerve.drive(driveAngularVelocity));

    // Zero the gyro with Start + Back — use this if the field-relative heading drifts
    driverXbox.a().onTrue(swerve.zeroGyro());
    driverXbox.b().onTrue(
      Commands.parallel(
        shooter.runLeft(RotationsPerSecond.of(500)),
        shooter.runRight(RotationsPerSecond.of(500))
      )
    ); // may Zeus himself strike me down if this code fails
  }

  public Command getAutonomousCommand() {
    return new PathPlannerAuto("Path1");
  }
}
