package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.local.SparkWrapper;

public class IntakeFrame extends SubsystemBase {

    private SmartMotorControllerConfig frameMotorConfig;
    private SmartMotorController frameMotor;
    private SparkMax frameMotorSpark;
    private ArmConfig frameConfig;
    private Arm frame;

    public IntakeFrame(){
        frameMotorConfig = IntakeConstants.intakeFrameMotorConfig.withSubsystem(this);
        frameMotorSpark = new SparkMax(IntakeConstants.intakeFrameMotor, MotorType.kBrushless);
        frameMotor = new SparkWrapper(
            frameMotorSpark,
            DCMotor.getNEO(1),
            frameMotorConfig);
        frameConfig = IntakeConstants.intakeFrameConfig;
        frame = new Arm(frameConfig, frameMotor);
    }
    
    public Command runTo(Angle angle, Angle tolerance){
        return frame.runTo(angle, tolerance);
    }
    public Command set(double dutyCycle){
        return frame.set(dutyCycle);
    }

    @Override
    public void periodic(){
        frame.updateTelemetry();
    }

    @Override
    public void simulationPeriodic(){
        frame.simIterate();
    }
}
