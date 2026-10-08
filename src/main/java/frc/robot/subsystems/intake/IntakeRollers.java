package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.shooter.Flywheel;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.local.SparkWrapper;

public class IntakeRollers extends SubsystemBase {

    private SmartMotorControllerConfig rollerMotorConfig;
    private SmartMotorController rollerMotor;
    private SparkMax rollerMotorSpark;
    private FlyWheelConfig rollerConfig;
    private FlyWheel roller;

    public IntakeRollers(){
        rollerMotorConfig = IntakeConstants.intakeRollerMotorConfig.withSubsystem(this);
        rollerMotorSpark = new SparkMax(IntakeConstants.intakeRollerMotor, MotorType.kBrushless);
        rollerMotor = new SparkWrapper(
            rollerMotorSpark,
            DCMotor.getNEO(1),
            rollerMotorConfig);
        rollerConfig = IntakeConstants.intakeRollerConfig;
        roller = new FlyWheel(rollerConfig, rollerMotor);
    }
    
    public Command run(AngularVelocity angle){
        return roller.run(angle);
    }
    public Command set(double dutyCycle){
        return roller.set(dutyCycle);
    }

    @Override
    public void periodic(){
        roller.updateTelemetry();
    }

    @Override
    public void simulationPeriodic(){
        roller.simIterate();
    }
}
