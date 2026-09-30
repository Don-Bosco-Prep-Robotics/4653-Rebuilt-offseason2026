package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RPM;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Flywheels extends SubsystemBase{

    private Flywheel left;
    private Flywheel right;
    private FlywheelsGoal goal;

    public Flywheels(Flywheel left, Flywheel Right){
        this.left = left;
        this.right = right;
    }

    public Command setGoal(FlywheelsGoal goal){
        return Commands.runOnce(() -> this.goal = goal)
        .andThen(Commands.select(
            Map.of(
                FlywheelsGoal.SHOOTING,
                this.shoot(),
                FlywheelsGoal.IDLE,
                this.stop()),
            ()->goal
            )).withName("Set Flywheel Goal");
    }

    public Command shoot(){
        return left.run(RPM.of(15)).alongWith(right.run(RPM.of(15)));
    }
    public Command stop(){
        return left.run(RPM.of(15)).alongWith(right.run(RPM.of(15)));
    }

    public enum FlywheelsGoal {
        SHOOTING,
        IDLE
    }
}
