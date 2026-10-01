package frc.robot.subsystems.superstructure;

import java.util.Map;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Flywheel;
import frc.robot.subsystems.shooter.Flywheels;
import frc.robot.subsystems.shooter.Flywheels.FlywheelsGoal;
import yams.mechanisms.velocity.FlyWheel;

public class Superstructure extends SubsystemBase {

    public Flywheels flywheels;
    public Intake intake;
    public Indexer indexer;
    
    private final Map<Goal, Supplier<Command>> goalCommands;
    private Goal goal = Goal.IDLE;

    public Superstructure (
        Flywheel flywheelLeft,
        Flywheel flywheelRight,
        Intake intake,
        Indexer indexer
    ){
        this.flywheels = new Flywheels(flywheelLeft, flywheelRight);
        this.intake = intake;
        this.indexer = indexer;

        goalCommands = Map.of(
            Goal.SCORING,
            () -> Commands.sequence(
                flywheels.setGoal(FlywheelsGoal.SHOOTING),
                indexer.setGoal(Indexer.IndexerGoal.INDEXING)
            ).withName("Start Scoring"),
            Goal.COLLECTING,
            () -> Commands.sequence(

            ).withName("Start Collecting"),
            Goal.IDLE,
            () -> Commands.sequence(
                flywheels.setGoal(FlywheelsGoal.IDLE),
                indexer.setGoal(Indexer.IndexerGoal.IDLE)
            ).withName("Start Idle")
        );

    }

    public Command setGoal(Goal goal) {
        return Commands.runOnce(() -> this.goal = goal, this).andThen(goalCommands.get(goal).get());
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }


    public static enum Goal {
        SCORING,
        COLLECTING,
        IDLE
    }
}
