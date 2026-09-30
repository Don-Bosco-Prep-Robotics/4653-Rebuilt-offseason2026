// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Amps;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  public static class ShooterConstants {

    //LEFT FLYWHEEL
    public static final int flywheelLeftMotor = 14;
    public final SmartMotorControllerConfig flywheelLeftMotorConfig = new SmartMotorControllerConfig()
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withClosedLoopController(0.00016541, 0, 0)
        .withSimClosedLoopController(0.00016541, 0, 0)
        .withFeedforward(new SimpleMotorFeedforward(0.27937, 0.089836, 0.014557))
        .withSimFeedforward(new SimpleMotorFeedforward(0.27937, 0.089836, 0.014557))
        .withTelemetry("FlywheelLeft Motor", TelemetryVerbosity.HIGH)
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(1, 1.17)))
        .withMotorInverted(false)
        .withIdleMode(MotorMode.COAST)
        .withStatorCurrentLimit(Amps.of(40));
    
    //RIGHT FLYWHEEL
    public static final int flywheelRightMotor = 13;
    public final SmartMotorControllerConfig flywheelRightMotorConfig = new SmartMotorControllerConfig()
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withClosedLoopController(0.00016541, 0, 0)
        .withSimClosedLoopController(0.00016541, 0, 0)
        .withFeedforward(new SimpleMotorFeedforward(0.27937, 0.089836, 0.014557))
        .withSimFeedforward(new SimpleMotorFeedforward(0.27937, 0.089836, 0.014557))
        .withTelemetry("FlywheelRight Motor", TelemetryVerbosity.HIGH)
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(1, 1.17)))
        .withMotorInverted(false)
        .withIdleMode(MotorMode.COAST)
        .withStatorCurrentLimit(Amps.of(40));
    
  }
}
