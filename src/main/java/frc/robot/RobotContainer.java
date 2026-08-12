// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.concurrent.atomic.AtomicBoolean;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import choreo.auto.AutoChooser;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CommandSwerveDrivetrainChoreo;
import frc.robot.subsystems.CommandSwerveDrivetrainPathPlanner;
import frc.robot.subsystems.Limelight;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private AtomicBoolean targetingAprilTag = new AtomicBoolean(false);
  private AtomicBoolean drivingRobotCentric = new AtomicBoolean(false);

  private static final double LIMELIGHT_FOV = 62.5d;

  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  public final Limelight limelight = new Limelight();

  public final CommandXboxController controller = new CommandXboxController(0);
  // public final Joystick gunnerPad = new Joystick(OperatorInterfaceConstants.gunnerPadPort);

  public RobotContainer() {
    configureBindings();
  }

  

  // Define triggers and their respective commands
  private void configureBindings() {
    limelight.setStreamMode(Limelight.STREAM_MODE_STANDARD);

    controller.b().onTrue(new InstantCommand(() -> drivingRobotCentric.set(!drivingRobotCentric.get())));
    controller.leftBumper().onTrue(new InstantCommand(() -> {drivetrain.seedFieldCentric();}));
    controller.x().whileTrue(drivetrain.applyRequest(() -> brake));
    controller.a().onTrue(new InstantCommand(() -> targetingAprilTag.set(!targetingAprilTag.get())));

    Trigger targetChange = new Trigger(() -> targetingAprilTag.get());
    Trigger toggleMode = new Trigger(() -> drivingRobotCentric.get());

    double MaxSpeed = DefaultDrivetrainCommand.MaxSpeed;
    double MaxAngularRate = DefaultDrivetrainCommand.MaxAngularRate;

    SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
              .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
              .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    Command target = drivetrain.applyRequest(() ->
        drive.withVelocityX(shape(controller.getLeftY()) * MaxSpeed)
        .withVelocityY(shape(controller.getLeftX()) * MaxSpeed)
        .withRotationalRate(shapeRotation(getRotation()) * MaxAngularRate)
    );

    DefaultDrivetrainCommand def = new DefaultDrivetrainCommand(drivetrain, controller, targetingAprilTag, drivingRobotCentric);

    targetChange.onTrue(new InstantCommand(() -> def.update(target)))
      .onFalse(new InstantCommand(() -> def.update()));

    toggleMode.onChange(new InstantCommand(() ->
      {
        if(!targetingAprilTag.get()) {
          def.update();
        }
      }
    ));

    drivetrain.setDefaultCommand(
      // Drivetrain will execute this command periodically
      def
    );

    limelight.setDefaultCommand(new DefaultLimelightCommand(limelight));

    Command toggleLEDs = new InstantCommand(() -> limelight.toggleLEDs());

    controller.y().onTrue(toggleLEDs);
  }

  public Command getAutonomousCommand() {
    return new InstantCommand();
  }

  public static double shape(double initial) {
    return initial * Math.abs(initial);
  }

  public static double shapeRotation(double initial) {
   return initial * Math.abs(initial);
  }

  // Helper method to resolve the rotational offset to a scale of -1 to 1
  public double getRotation() {
    return limelight.horizontalOffsetFromCrosshairAsDegrees() * 2 / LIMELIGHT_FOV;
  }

  public double getDistanceFromAprilTagInches() {
    return limelight.getDistanceFromAprilTagInches();
  }

  public double getHorizontalDegreeOffset() {
    return limelight.horizontalOffsetFromCrosshairAsDegrees();
  }
}
