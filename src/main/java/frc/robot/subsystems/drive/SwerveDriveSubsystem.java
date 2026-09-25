package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Degrees;

import java.io.File;
import java.util.function.DoubleSupplier;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.parser.SwerveParser;
import swervelib.parser.SwerveParser.SwerveDriveDevices;
import yams.mechanisms.config.SwerveDriveConfig;
import yams.mechanisms.swerve.SwerveDrive;
import yams.mechanisms.swerve.utility.SwerveInputStream;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.telemetry.SwerveDriveTelemetryConfig;

public class SwerveDriveSubsystem extends SubsystemBase
{

  private SwerveDrive drive;  
  private final AHRS gyro = new AHRS(NavXComType.kMXP_SPI);

  public SwerveDriveSubsystem()
  {
    SmartDashboard.putData(this);
    var cfg = new SwerveDriveConfig()
        .withStartingPose(new Pose2d(3, 3, Rotation2d.kZero))
        .withSubsystem(this)
        .withTranslationController(new PIDController(4, 0, 0))
        .withRotationController(new PIDController(1, 0, 0))
        .withTelemetry("swerve", new SwerveDriveTelemetryConfig(TelemetryVerbosity.HIGH))
        .withGyro(() -> Degrees.of(-gyro.getAngle()));

    SwerveParser.parse(new File(Filesystem.getDeployDirectory(), "swerve/base"));
    SwerveDriveDevices devices = SwerveParser.createSwerveDriveDevices(cfg);
    drive = devices.swerveDrive();
     // Set the absolute encoder to be used over the internal encoder and push the offsets onto it. Throws warning if not possible
    // You can also create the SwerveDrive without the ability to retrieve the devices like this.
    // drive = SwerveParser.createSwerveDrive(cfg);
  
  }

  public SwerveInputStream getAngularVelocityStream(DoubleSupplier x, DoubleSupplier y,
                                                    DoubleSupplier rot)
  {
    return new SwerveInputStream(drive, x, y, rot);
  }

  public Command drive(SwerveInputStream stream)
  {
    return drive.drive(() -> ChassisSpeeds.fromFieldRelativeSpeeds(stream.get(),
                                                                   new Rotation2d(drive.getGyroAngle())));
  }

  /** Zero the gyro heading. Bind this to a button combo for field recovery. */
  public Command zeroGyro()
  {
    return runOnce(() -> drive.zeroGyro());
  }

  @Override
  public void periodic()
  {
    drive.updateTelemetry();
  }

  @Override
  public void simulationPeriodic()
  {
    drive.simIterate();
  }
}