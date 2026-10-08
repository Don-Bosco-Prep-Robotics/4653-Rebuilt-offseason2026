package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import java.util.Map;

import org.littletonrobotics.junction.AutoLogOutput;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.indexer.Indexer.IndexerGoal;
import frc.robot.subsystems.shooter.Flywheel;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.positional.Arm;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.local.SparkWrapper;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IntakeConstants;;

public class Intake extends SubsystemBase {

    private IntakeFrame frame;
    private IntakeRollers rollers;

    @AutoLogOutput
    private IntakeGoal goal;
    
    public Intake(){
        this.frame = new IntakeFrame();
        this.rollers = new IntakeRollers();
    }

    public Command setGoalCommand(IntakeGoal goal) {
        return Commands.runOnce(() -> this.goal = goal)
            .andThen(Commands.select(
                Map.of(
                    IntakeGoal.INTAKING,
                    Commands.parallel(
                        frame.runTo(Degrees.of(80), Degrees.of(5)).andThen(frame.set(0)),
                        rollers.run(RPM.of(55))
                    ),
                    IntakeGoal.IDLE,
                    Commands.parallel(
                        frame.set(0),
                        rollers.set(0)
                    ),
                    IntakeGoal.STOWED,
                    Commands.parallel(
                        frame.runTo(Degrees.of(10), Degrees.of(2)).andThen(frame.set(0)),
                        rollers.set(0)
                    )
                ),
                () -> goal
            ))
            .withName("Set Intake Goal");
    }

    public Command testArmFromExtendedCommand(){
        return frame.runTo(Degrees.of(80), Degrees.of(3));//.andThen(frame.set(0));
    }

    @Override public void periodic() {

    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    public enum IntakeGoal {
        STOWED,
        INTAKING,
        OUTTAKING,
        SHAKING,
        IDLE
    }
}
