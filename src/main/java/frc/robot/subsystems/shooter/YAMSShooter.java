package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;
import frc.robot.Constants.ShooterConstants;

public class YAMSShooter extends SubsystemBase {
    
    private SmartMotorControllerConfig shooterMotorConfig = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withClosedLoopController(1, 0, 0)
        .withSimClosedLoopController(1, 0, 0)
        .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
        .withSimFeedforward(new SimpleMotorFeedforward(0, 0, 0))
        .withTelemetry("ShooterMotor", TelemetryVerbosity.HIGH)
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
        .withMotorInverted(false)
        .withIdleMode(MotorMode.COAST)
        .withStatorCurrentLimit(Amps.of(40));

    


    
    private SparkMax shooterLeftSpark = new SparkMax(ShooterConstants.shooterLeftMotor, MotorType.kBrushless);
    private SmartMotorController shooterLeftSmartSpark = new SparkWrapper(
        shooterLeftSpark, 
        DCMotor.getNEO(1), 
        shooterMotorConfig);

    private SparkMax shooterRightSpark = new SparkMax(ShooterConstants.shooterRightMotor, MotorType.kBrushless);
    private SmartMotorController shooterRightSmartSpark = new SparkWrapper(
        shooterRightSpark, 
        DCMotor.getNEO(1), 
        shooterMotorConfig.withLooselyCoupledFollowers(shooterLeftSmartSpark));
    
    private final FlyWheelConfig shooterConfig = new FlyWheelConfig()
        .withDiameter(Inches.of(4))
        .withTelemetry("ShooterMech", TelemetryVerbosity.HIGH);

    private FlyWheel shooterLeft = new FlyWheel(shooterConfig, shooterLeftSmartSpark);
    private FlyWheel shooterRight = new FlyWheel(shooterConfig, shooterRightSmartSpark);


    public AngularVelocity getVelocity() {return shooterLeft.getSpeed();}

    //public Command runLeft(AngularVelocity speed) {return shooterLeft.run(speed);}
    public Command runRight(AngularVelocity speed) {return shooterRight.run(speed);}

    public void setLeftVelocitySetpoint(AngularVelocity speed) {shooterLeft.setMechanismVelocitySetpoint(speed);}
    public void setRightVelocitySetpoint(AngularVelocity speed) {shooterRight.setMechanismVelocitySetpoint(speed);}

//hi
    @Override
    public void periodic() {
        shooterLeft.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        shooterLeft.simIterate();
    }


}
