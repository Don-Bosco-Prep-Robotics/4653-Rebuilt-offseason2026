package frc.robot.subsystems.shooter;

import com.revrobotics.spark.SparkMax;

import static edu.wpi.first.units.Units.RPM;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
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

    public Flywheel(SmartMotorControllerConfig shooterMotorConfig, FlyWheelConfig shooterConfig, int sparkMaxID, String name){

        super(name);

        this.shooterMotorConfig = shooterMotorConfig.withSubsystem(this);
        this.shooterSpark = new SparkMax(sparkMaxID, MotorType.kBrushless);
        this.shooterSmartSpark = new SparkWrapper(
            this.shooterSpark, 
            DCMotor.getNEO(1), 
            this.shooterMotorConfig);
        this.shooterConfig = shooterConfig;
        this.shooter = new FlyWheel(this.shooterConfig, this.shooterSmartSpark);
    }

    //This whole thing is very very janky and I am going to clean it up
    //public Command runLeft(AngularVelocity speed) {return shooterLeft.run(speed);}
    // public void run(AngularVelocity speed) {
    //     CommandScheduler.getInstance().schedule(shooter.run(speed));}
    // public void stop() {
    //     CommandScheduler.getInstance().schedule(shooter.set(0));}

    public Command run(AngularVelocity speed) {
        return shooter.run(speed);}
    public Command stop() {
        return shooter.set(0);}
    public void shootVoid(){
        Commands.runOnce(() -> shooter.run(RPM.of(15)));
    }

    public void set(double dutyCycle) {
        shooter.set(dutyCycle);
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
