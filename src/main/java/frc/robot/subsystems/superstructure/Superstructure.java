package frc.robot.subsystems.superstructure;

import java.util.Map;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    @AutoLogOutput
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
                flywheels.setGoalCommand(FlywheelsGoal.SHOOTING),
                this.indexer.setGoalCommand(Indexer.IndexerGoal.INDEXING)
            ).withName("Start Scoring"),
            Goal.COLLECTING,
            () -> Commands.sequence(

            ).withName("Start Collecting"),
            Goal.IDLE,
            () -> Commands.sequence(
                flywheels.setGoalCommand(FlywheelsGoal.IDLE),
                this.indexer.setGoalCommand(Indexer.IndexerGoal.IDLE)
            ).withName("Start Idle")
        );

    }

    public Command setGoal(Goal newGoal) {
        return Commands.runOnce(() -> this.goal = newGoal, this)
            .andThen(goalCommands.get(newGoal).get())
            .withName("Superstructure Set Goal");
    }

    @Override
    public void periodic() {
        // switch (this.goal) {
        //     case SCORING -> {
        //         flywheels.setGoal(FlywheelsGoal.SHOOTING);
        //         indexer.setGoal(Indexer.IndexerGoal.INDEXING);
        //     }
        //     case COLLECTING -> {
        //         flywheels.setGoal(FlywheelsGoal.IDLE);
        //         // indexer.setGoal(Indexer.IndexerGoal);
        //     }
        //     case IDLE -> {
        //         flywheels.setGoal(FlywheelsGoal.IDLE);
        //         indexer.setGoal(Indexer.IndexerGoal.IDLE);
        // }
        // }
        Logger.recordOutput(
                "Superstructure/Current Command",
                this.getCurrentCommand() == null
                        ? "None"
                        : this.getCurrentCommand().getName());
    }


    public static enum Goal {
        SCORING,
        COLLECTING,
        IDLE
    }
    
}
