package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

    private Mechanism2d mechanism = new Mechanism2d(1,1);
    private MechanismRoot2d mechroot = mechanism.getRoot("Intake Frame Mech", 0, 0);
    private MechanismLigament2d frameLigament;

    public IntakeFrame(){
        frameMotorConfig = IntakeConstants.intakeFrameMotorConfig.withSubsystem(this);
        frameMotorSpark = new SparkMax(IntakeConstants.intakeFrameMotor, MotorType.kBrushless);
        frameMotor = new SparkWrapper(
            frameMotorSpark,
            DCMotor.getNEO(1),
            frameMotorConfig);
        frameConfig = IntakeConstants.intakeFrameConfig;
        frame = new Arm(frameConfig, frameMotor);

        frameLigament = mechroot.append(new MechanismLigament2d("Intake Frame Ligament", 1, 90));
        SmartDashboard.putData("Mechanisms/Inake Frame/Mech2d", mechanism);

    }
    
    public Command runTo(Angle angle, Angle tolerance){
        return frame.runTo(angle, tolerance);
    }
    public Command set(double dutyCycle){
        return frame.set(dutyCycle);
    }
    public Command setEncoder(Angle angle){
        return Commands.runOnce(()->{frameMotor.setEncoderPosition(angle);}, this);
    }

    @Override
    public void periodic(){
        frame.updateTelemetry();
        frameLigament.setAngle(frame.getAngle().in(Degrees) + 90);
    }

    @Override
    public void simulationPeriodic(){
        frame.simIterate();
    }
}
