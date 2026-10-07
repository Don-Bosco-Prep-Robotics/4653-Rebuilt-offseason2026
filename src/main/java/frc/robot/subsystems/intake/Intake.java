package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import java.util.Map;

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

    private SmartMotorControllerConfig intakeRollerMotorController;
    private SmartMotorControllerConfig intakeFrameMotorController;
    private SmartMotorController intakeRollerMotor;
    private SmartMotorController intakeFrameMotor;
    private SparkMax intakeRollerSpark;
    private SparkMax intakeFrameSpark;
    private ArmConfig intakeFrameConfig;
    private Arm intakeFrame;
    private FlyWheelConfig intakeRollerConfig;
    private FlyWheel intakeRoller;

    private IntakeGoal goal = IntakeGoal.STOWED;
    
    public Intake(){
        //Intake Roller
        intakeRollerMotorController =IntakeConstants.intakeRollerMotorConfig.withSubsystem(this);
        intakeRollerSpark = new SparkMax(IntakeConstants.intakeRollerMotor, MotorType.kBrushless);
        intakeRollerMotor = new SparkWrapper(
            intakeRollerSpark,
            DCMotor.getNEO(1), 
            intakeRollerMotorController);
        intakeRollerConfig = IntakeConstants.intakeRollerConfig;
        intakeRoller = new FlyWheel(intakeRollerConfig, intakeRollerMotor);

        //Intake Frame
        intakeFrameMotorController = IntakeConstants.intakeFrameMotorConfig.withSubsystem(this);
        intakeFrameSpark = new SparkMax(IntakeConstants.intakeFrameMotor, MotorType.kBrushless);
        intakeFrameMotor = new SparkWrapper(
            intakeFrameSpark,
            DCMotor.getNEO(1),
            intakeFrameMotorController);
        intakeFrameConfig = IntakeConstants.intakeFrameConfig;
        intakeFrame = new Arm(intakeFrameConfig, intakeFrameMotor);
    } 

    public Command setGoalCommand(IntakeGoal goal) {
        return Commands.runOnce(() -> this.goal = goal)
            .andThen(Commands.select(
                Map.of(
                    IntakeGoal.INTAKING,
                    Commands.parallel(
                        intakeFrame.runTo(Degrees.of(80), Degrees.of(5)).andThen(intakeFrame.set(0))//,
                        // intakeRoller.run(RPM.of(55))
                    ),
                    IntakeGoal.IDLE,
                    Commands.parallel(
                        intakeFrame.set(0)//,
                        // intakeRoller.set(0)
                    ),
                    IntakeGoal.STOWED,
                    Commands.parallel(
                        intakeFrame.runTo(Degrees.of(10), Degrees.of(2)).andThen(intakeFrame.set(0))//,
                        // intakeRoller.set(0)
                    )
                ),
                () -> goal
            ))
            .withName("Set Intake Goal");
    }

    public Command testArmFromExtendedCommand(){
        return intakeFrame.runTo(Degrees.of(80), Degrees.of(3));//.andThen(intakeFrame.set(0));
    }

    @Override public void periodic() {
        intakeFrame.updateTelemetry();
        intakeRoller.updateTelemetry();
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
