package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RPM;

import java.util.Map;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.mechanisms.velocity.FlyWheel;

public class Flywheels extends SubsystemBase{

    private Flywheel left;
    private Flywheel right;

    @AutoLogOutput
    private FlywheelsGoal goal = FlywheelsGoal.IDLE;

    public Flywheels(Flywheel left, Flywheel right){
        this.left = left;
        this.right = right;
    }

    public Command setGoalCommand(FlywheelsGoal goal){
        return Commands.runOnce(() -> this.goal = goal).withName("Flywheel Set Goal")
        .andThen(Commands.select(
            Map.of(
                FlywheelsGoal.SHOOTING,
                this.shoot(),
                FlywheelsGoal.IDLE,
                this.stop()),
            ()->goal
            )).withName("Set Flywheel Goal");
    }

    public void setGoal(FlywheelsGoal goal){
        this.goal = goal;
    }

    public FlywheelsGoal getGoal(){
        return this.goal;
    }

    public Command shoot(){
        return left.run(RPM.of(100)).alongWith(right.run(RPM.of(100)));
    }
    public Command stop(){
        return left.set(0).alongWith(right.set(0));
    }
    

    @Override
    public void periodic() {
        // switch (goal) {
        //     case SHOOTING -> {
        //         Commands.runOnce(() -> this.shoot());
        //     }
        //     case IDLE -> {
        //         Commands.runOnce(() -> this.stop());
        //     }
        // }
    }

    public enum FlywheelsGoal {
        SHOOTING,
        IDLE
    }
}
