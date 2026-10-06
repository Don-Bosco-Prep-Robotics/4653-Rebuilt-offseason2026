// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RPM;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

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

    //TODO - Figure out why the livetuning doesn't appear to make a difference
    //TODO - Figure out why the flyWheelLeft Values are showing up as 1 on the tuner
    //LEFT FLYWHEEL
    public static final int flywheelLeftMotor = 14;
    public static final SmartMotorControllerConfig flywheelLeftMotorConfig = new SmartMotorControllerConfig()
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
    // public static final SparkWrapper flywheelLeftSmartSpark = new SparkWrapper(
    //         new SparkMax(flywheelLeftMotor, MotorType.kBrushless), 
    //         DCMotor.getNEO(1), 
    //         flywheelLeftMotorConfig);
    public static final FlyWheelConfig flywheelLeftConfig = new FlyWheelConfig()
        .withDiameter(Inches.of(4))
        .withTelemetry("FlywheelLeft", TelemetryVerbosity.HIGH);

    //RIGHT FLYWHEEL
    public static final int flywheelRightMotor = 13;
    public static final SmartMotorControllerConfig flywheelRightMotorConfig = new SmartMotorControllerConfig()
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withClosedLoopController(0.00016541, 0, 0)
        .withSimClosedLoopController(0.00016541, 0, 0)
        .withFeedforward(new SimpleMotorFeedforward(0.27937, 0.089836, 0.014557))
        .withSimFeedforward(new SimpleMotorFeedforward(0.27937, 0.089836, 0.014557))
        .withTelemetry("FlywheelRight Motor", TelemetryVerbosity.HIGH)
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(1, 1.17)))
        .withMotorInverted(true)
        .withIdleMode(MotorMode.COAST)
        .withStatorCurrentLimit(Amps.of(40));
    // public static final SparkWrapper flywheelRightSmartSpark = new SparkWrapper(
    //         new SparkMax(flywheelRightMotor, MotorType.kBrushless), 
    //         DCMotor.getNEO(1), 
    //         flywheelRightMotorConfig);
    public static final FlyWheelConfig flywheelRightConfig = new FlyWheelConfig()
        .withDiameter(Inches.of(4))
        .withTelemetry("FlywheelRight", TelemetryVerbosity.HIGH);
    
  }
  public static class IndexerConstants {
    public static final int indexerMotor = 20;

    //TODO - Figure out why this isn't turning at all - SOLVED - duplicate CAN ID with the REVPD
    public static final SmartMotorControllerConfig indexerMotorConfig = new SmartMotorControllerConfig()
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withFeedforward(new SimpleMotorFeedforward(1, 1, 1))
        .withSimFeedforward(new SimpleMotorFeedforward(1, 1, 1))
        .withTelemetry("Indexer Motor", TelemetryVerbosity.HIGH)
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(1, 1)))
        .withMotorInverted(false)
        .withIdleMode(MotorMode.COAST)
        .withStatorCurrentLimit(Amps.of(40));
    public static final FlyWheelConfig indexerConfig = new FlyWheelConfig()
        .withDiameter(Inches.of(4))
        .withTelemetry("Indexer", TelemetryVerbosity.HIGH);

    public static final AngularVelocity indexingSpeed = RPM.of(10);
  }
  
}
