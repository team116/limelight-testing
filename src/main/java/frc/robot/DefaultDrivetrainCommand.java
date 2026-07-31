package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.concurrent.atomic.AtomicBoolean;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class DefaultDrivetrainCommand extends Command {
    private final static String yawKey = "Yaw from AprilTag";
    private final static String normalizedYawKey = "Normalized Yaw from AprilTag";
    private final static String powerKey = "Rotational Power";

    private AtomicBoolean targetingAprilTag = new AtomicBoolean(false);
    private AtomicBoolean drivingRobotCentric = new AtomicBoolean(false);

    private final static double kP_YAW = 1; // FIXME: Tune
    private final static double LIMELIGHT_FOV = Math.toRadians(62.5); // in degrees

    // NOTE: Ideal value is division by 2.0 for optimal driver.  4.0 is too slow
    public static double MaxSpeed = (TunerConstants.kSpeedAt12Volts.in(MetersPerSecond)) / 2.0d; // kSpeedAt12Volts desired top speed
    public static double MaxAngularRate = (RotationsPerSecond.of(0.75).in(RadiansPerSecond)) / 2.0d; // 3/4 of a rotation per second max angular velocity

    public static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final CommandSwerveDrivetrain drivetrain;

    private final Command driveRobotCentric;
    private final Command driveFieldCentric;

    private Command defaultCommand;

    public DefaultDrivetrainCommand(CommandSwerveDrivetrain drivetrain, CommandXboxController controller, AtomicBoolean targetingAprilTag, AtomicBoolean drivingRobotCentric) {
        this.drivetrain = drivetrain;
        // uncomment this next line whenever we can confirm that the tag targeting works and is useful
        // controller.a().onTrue(new InstantCommand(() -> targetingAprilTag.set(!targetingAprilTag.get())));
        this.targetingAprilTag = targetingAprilTag;
        this.drivingRobotCentric = drivingRobotCentric;

        driveRobotCentric = drivetrain.applyRequest(() ->
                    robotCentric.withVelocityX(shape(controller.getLeftY()) * MaxSpeed)
                        .withVelocityY(shape(controller.getLeftX()) * MaxSpeed)
                        .withRotationalRate(shapeRotation(controller.getRightX()) * MaxAngularRate)
                );
        
        driveFieldCentric = drivetrain.applyRequest(() -> 
                    drive.withVelocityX(shape(controller.getLeftY()) * MaxSpeed)
                        .withVelocityY(shape(controller.getLeftX()) * MaxSpeed)
                        .withRotationalRate(shapeRotation(controller.getRightX()) * MaxAngularRate)
                );
        super.addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        update();
        defaultCommand.execute();
    }

    public void update(Command newDefault) {
        if(newDefault==null) {
            if(drivingRobotCentric.get()) {
                defaultCommand = driveRobotCentric;
            } else {
                defaultCommand = driveFieldCentric;
            }
        } else {
            defaultCommand = newDefault;
        }
    }

    public void update() {
        update(null);
    }

    public static double shape(double initial) {
        return initial * Math.abs(initial);
    }

    public static double shapeRotation(double initial) {
        return initial * Math.abs(initial);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(new SwerveRequest.Idle()); // Idle out our motors if we want to stop the command (for safety's sake)
    }
}
