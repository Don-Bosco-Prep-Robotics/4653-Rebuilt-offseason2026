package frc.robot.subsystems.shooter;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.local.SparkWrapper;

public class Flywheel extends SubsystemBase {
    
    private SmartMotorControllerConfig shooterMotorConfig;
    private SparkMax shooterSpark;
    private SmartMotorController shooterSmartSpark;
    private FlyWheelConfig shooterConfig;
    private FlyWheel shooter;


    public AngularVelocity getVelocity() {return shooter.getSpeed();}

    public Flywheel(SmartMotorControllerConfig shooterMotorConfig, FlyWheelConfig shooterConfig, int sparkMaxID){
        this.shooterMotorConfig = shooterMotorConfig.withSubsystem(this);
        shooterSpark = new SparkMax(sparkMaxID, MotorType.kBrushless);
        shooterSmartSpark = new SparkWrapper(
            shooterSpark, 
            DCMotor.getNEO(1), 
            shooterMotorConfig);
        this.shooterConfig = shooterConfig;
        this.shooter = new FlyWheel(shooterConfig, shooterSmartSpark);
    }

    //public Command runLeft(AngularVelocity speed) {return shooterLeft.run(speed);}
    public Command run(AngularVelocity speed) {return shooter.run(speed);}
    public Command stop() {return shooter.set(0);}
    public Command set(double dutyCycle) {
        return shooter.set(dutyCycle);
    }

    public void setVelocitySetpoint(AngularVelocity speed) {shooter.setMechanismVelocitySetpoint(speed);}


    @Override
    public void periodic() {
        shooter.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        shooter.simIterate();
    }


}
