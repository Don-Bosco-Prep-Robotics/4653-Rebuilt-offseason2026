package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.io.File;
import java.io.IOException;
import java.util.function.DoubleSupplier;

import javax.sound.sampled.Control;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.LoggedTunableNumber;
import frc.robot.utils.TunableControls;
import frc.robot.utils.TunableControls.ControlConstants;
import frc.robot.utils.TunableControls.TunableControlConstants;
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
  private LoggedTunableNumber translationP = new LoggedTunableNumber("/swerve/translation/P", 4);
  private LoggedTunableNumber translationI = new LoggedTunableNumber("/swerve/translation/I", 0);
  private LoggedTunableNumber translationD = new LoggedTunableNumber("/swerve/translation/D", 0);
  private LoggedTunableNumber rotationP = new LoggedTunableNumber("/swerve/rotation/P", 4);
  private LoggedTunableNumber rotationI = new LoggedTunableNumber("/swerve/rotation/I", 0);
  private LoggedTunableNumber rotationD = new LoggedTunableNumber("/swerve/rotation/D", 0);

  public SwerveDriveSubsystem()
  {
    SmartDashboard.putData(this);
    var cfg = new SwerveDriveConfig()
        .withStartingPose(Pose2d.kZero)
        .withSubsystem(this)
        .withTranslationController(new PIDController(translationP.getAsDouble(), translationI.getAsDouble(), translationD.getAsDouble()))
        .withRotationController(new PIDController(rotationP.getAsDouble(), rotationI.getAsDouble(), rotationD.getAsDouble()))
        // .withTranslationController(
        //   new TunableControls.TunablePIDController(
        //     new TunableControlConstants(
        //       getName(),
        //       new ControlConstants().withPID(4, 0, 0))))
        .withTelemetry("swerve", new SwerveDriveTelemetryConfig(TelemetryVerbosity.HIGH))
        .withMaximumChassisSpeed(MetersPerSecond.of(4), DegreesPerSecond.of(360));
        // .withGyro(() -> Degrees.of(-gyro.getAngle()));

    SwerveParser.parse(new File(Filesystem.getDeployDirectory(), "swerve/base"));
    SwerveDriveDevices devices = SwerveParser.createSwerveDriveDevices(cfg);
    drive = devices.swerveDrive();

    try {
      setupPathPlanner();
    } catch (IOException | ParseException e) {
      throw new RuntimeException(
          "PathPlanner setup failed -- check deploy/pathplanner/settings.json exists and is formatted correctly", e);
    }
     // Set the absolute encoder to be used over the internal encoder and push the offsets onto it. Throws warning if not possible
    // You can also create the SwerveDrive without the ability to retrieve the devices like this.
    // drive = SwerveParser.createSwerveDrive(cfg);
  
  }
  private void setupPathPlanner() throws IOException, ParseException {
    AutoBuilder.configure(
        drive::getPose,                  // robot pose supplier
        drive::resetOdometry,             // called if an auto defines a starting pose
        drive::getRobotRelativeSpeed,     // ChassisSpeeds supplier -- MUST be robot-relative
        (speedsRobotRelative, moduleFeedForwards) ->
            drive.setRobotRelativeChassisSpeeds(speedsRobotRelative),
        new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.0, 0.0),  // translation PID
            new PIDConstants(5.0, 0.0, 0.0)   // rotation PID
        ),
        RobotConfig.fromGUISettings(),    // reads deploy/pathplanner/settings.json
        () -> {
          // Field origin is always the blue alliance wall -- flip paths when on red.
          var alliance = DriverStation.getAlliance();
          return alliance.filter(a -> a == DriverStation.Alliance.Red).isPresent();
        },
        this                              // subsystem requirement for the generated commands
    );
  }

  public SwerveInputStream getAngularVelocityStream(DoubleSupplier x, DoubleSupplier y,
                                                    DoubleSupplier rot) {


    boolean flipped = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;
    DoubleSupplier xInv = () -> -x.getAsDouble();
    DoubleSupplier yInv = () -> -y.getAsDouble();
    DoubleSupplier rotInv = () -> -rot.getAsDouble();

    // if(flipped){
    //   xInv = () -> x.getAsDouble();
    //   yInv = () -> y.getAsDouble();
    //   rotInv = () -> -rot.getAsDouble();
    // }

    return new SwerveInputStream(drive, xInv, yInv, rotInv)
    .withDeadband(0.05)
    .withMaximumLinearVelocity(MetersPerSecond.of(4))
    .withMaximumAngularVelocity(DegreesPerSecond.of(360));
  }

  public Command drive(SwerveInputStream stream)
  {
    return drive.drive(() -> ChassisSpeeds.fromFieldRelativeSpeeds(stream.get(), new Rotation2d(drive.getGyroAngle())));
    // return drive.drive(stream);
  }
  public Command driveFieldRelative(SwerveInputStream stream){
    return run(() -> drive.setFieldRelativeChassisSpeeds(stream.get())).withName("Drive Field Relative");
  }
  public Command tuningDrive(){
    return run(() -> drive.setRobotRelativeChassisSpeeds(
      new ChassisSpeeds(
        MetersPerSecond.of(1), 
        MetersPerSecond.of(0), 
        RotationsPerSecond.of(0)
      )
    ));
  }

  /** Zero the gyro heading. Bind this to a button combo for field recovery. */
  public Command zeroGyro()
  {
    return runOnce(() -> drive.zeroGyro());
  }

  @Override
  public void periodic() {
    drive.updateTelemetry();
    if(translationP.hasChanged(translationP.hashCode()) || translationD.hasChanged(translationD.hashCode()) || translationI.hasChanged(translationI.hashCode())
      || rotationP.hasChanged(rotationP.hashCode()) || rotationI.hasChanged(rotationI.hashCode()) || rotationD.hasChanged(rotationD.hashCode())
    ){
      drive.setTranslationPID(new PIDController(translationP.getAsDouble(), translationI.getAsDouble(), translationD.getAsDouble()));
      drive.setRotationPID(new PIDController(rotationP.getAsDouble(), rotationI.getAsDouble(), rotationD.getAsDouble()));
    }
  }

  @Override
  public void simulationPeriodic()
  {
    drive.simIterate();
  }
}