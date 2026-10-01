package frc.robot.subsystems.indexer;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import java.util.Map;
import java.util.function.Supplier;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.local.SparkWrapper;
import frc.robot.Constants.IndexerConstants;


public class Indexer extends SubsystemBase {
    
    private SmartMotorControllerConfig indexerMotorConfig;
    private SparkMax indexerSpark;
    private SmartMotorController indexerSmartSpark;
    private FlyWheelConfig indexerConfig;
    private FlyWheel indexer;

    private IndexerGoal goal = IndexerGoal.IDLE;

    public Indexer() {
        this.indexerMotorConfig = IndexerConstants.indexerMotorConfig.withSubsystem(this);
        this.indexerSpark = new SparkMax(IndexerConstants.indexerMotor, MotorType.kBrushless);
        this.indexerSmartSpark = new SparkWrapper(
            this.indexerSpark, 
            DCMotor.getNEO(1), 
            this.indexerMotorConfig);
        this.indexerConfig = IndexerConstants.indexerConfig;
        this.indexer = new FlyWheel(this.indexerConfig, this.indexerSmartSpark);
    }

    public Command run(AngularVelocity speed) {return indexer.run(speed);}
    public Command stop() {return indexer.set(0);}
    public Command set(double dutyCycle) {
        return indexer.set(dutyCycle);
    }


    public Command setGoal(IndexerGoal goal) {
        return Commands.runOnce(() -> this.goal = goal)
        .andThen(Commands.select(
            Map.of(
                IndexerGoal.INDEXING,
                this.run(IndexerConstants.indexingSpeed),
                IndexerGoal.IDLE,
                this.stop()),
            ()->goal
            )).withName("Set Indexer Goal");
    }

    @Override
    public void periodic() {
        indexer.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        indexer.simIterate();
    }


    public enum IndexerGoal {
        INDEXING,
        IDLE
    }

}
