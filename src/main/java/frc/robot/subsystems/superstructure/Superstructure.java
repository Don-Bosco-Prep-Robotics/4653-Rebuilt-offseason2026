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
import frc.robot.subsystems.indexer.Indexer.IndexerGoal;
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
        Indexer inputIndexer
    ){
        this.flywheels = new Flywheels(flywheelLeft, flywheelRight);
        this.intake = intake;
        this.indexer = inputIndexer;

        goalCommands = Map.of(
            Goal.SCORING,
            () -> Commands.parallel(
                flywheels.setGoalCommand(FlywheelsGoal.SHOOTING),
                indexer.setGoalCommand(IndexerGoal.INDEXING)
            ).withName("Start Scoring"),
            Goal.INTAKING,
            () -> Commands.parallel(

            ).withName("Start Collecting"),
            Goal.OUTTAKING,
            () -> Commands.parallel(
                indexer.setGoalCommand(IndexerGoal.OUTTAKING)
            ),
            Goal.IDLE,
            () -> Commands.parallel(
                flywheels.setGoalCommand(FlywheelsGoal.IDLE),
                indexer.setGoalCommand(IndexerGoal.IDLE)
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
        Logger.recordOutput(
                "Superstructure/Current Command",
                this.getCurrentCommand() == null
                        ? "None"
                        : this.getCurrentCommand().getName());
    }


    public static enum Goal {
        SCORING,
        INTAKING,
        OUTTAKING,
        IDLE
    }
    
}
