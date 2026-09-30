package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.Flywheel;

public class Superstructure extends SubsystemBase {

    public Flywheel flywheelLeft;
    public Flywheel flywheelRight;
    
    public Superstructure (
        Flywheel flywheelLeft,
        Flywheel flywheelRight
    ){

    }
}
