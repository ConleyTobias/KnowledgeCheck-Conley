package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.utility.*;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.autos.MaxSpeed;
import frc.robot.autos.RampSpeed;
import frc.robot.autos.SetpointsAndSpeeds;
import frc.robot.commands.WristManual;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIOCTRE;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveIOCTRE;
import frc.robot.subsystems.drive.requests.SwerveSetpointGen;
import frc.robot.subsystems.talonFX.Motor;
import frc.robot.subsystems.talonFX.MotorIOSim;
import frc.robot.utils.TunableController;
import frc.robot.utils.TunableController.TunableControllerType;

public class RobotContainer {
  private LinearVelocity MaxSpeed = TunerConstants.kSpeedAt12Volts;
  private final Motor motor;
  private final Arm arm;

  private final TunableController joystick =
      new TunableController(0)
          .withControllerType(TunableControllerType.QUADRATIC)
          .withDeadband(0.1);

  public final Drive drivetrain;
  // CTRE Default Drive Request
  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed.times(0.1))
          .withRotationalDeadband(Constants.MaxAngularRate.times(0.1)) // Add a 10% deadband
      ; // deleted ".withDriveRequestType(DriveRequestType.OpenLoopVoltage)"

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  public RobotContainer() {
    motor = new Motor(new MotorIOSim());
    arm = new Arm(new ArmIOCTRE());
    DriveIOCTRE currentDriveTrain = TunerConstants.createDrivetrain();
    drivetrain = new Drive(currentDriveTrain);

    configureBindings();
  }

  private void configureBindings() {
    // motor.setDefaultCommand(new SpeedControl(motor, () -> joystick.getLeftY()));
    arm.setDefaultCommand(new WristManual(arm, () -> joystick.getLeftY()));
    joystick.a().onTrue(Commands.runOnce(() -> motor.runVelocity(5)));
    joystick.b().onTrue(Commands.runOnce(() -> motor.runVelocity(0)));
    joystick.x().onTrue(Commands.runOnce(() -> motor.runPosition(0)));
    joystick.y().onTrue(Commands.runOnce(() -> motor.runPosition(1)));
    joystick.leftBumper().onTrue(Commands.runOnce(() -> arm.runClawPosition(0)));
    joystick.rightBumper().onTrue(Commands.runOnce(() -> arm.runClawPosition(1)));

    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(
            () ->
                drive
                    .withVelocityX(
                        MaxSpeed.times(
                            -joystick
                                .customLeft()
                                .getY())) // Drive forward with negative Y (forward)
                    .withVelocityY(
                        MaxSpeed.times(
                            -joystick.customLeft().getX())) // Drive left with negative X (left)
                    .withRotationalRate(
                        Constants.MaxAngularRate.times(
                            -joystick
                                .customRight()
                                .getX())))); // Drive counterclockwise with negative X (left)

    // joystick.a().onTrue(Commands.runOnce(() -> drivetrain.resetPose(Pose2d.kZero)));
    joystick
        .b()
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    point.withModuleDirection(
                        new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // Custom Swerve Request that use PathPlanner Setpoint Generator. Tuning NEEDED. Instructions
    // can be found here
    // https://hemlock5712.github.io/Swerve-Setup/talonfx-swerve-tuning.html
    SwerveSetpointGen setpointGen =
        new SwerveSetpointGen(
            drivetrain.getChassisSpeeds(),
            drivetrain.getModuleStates(),
            drivetrain::getRotation); // Deleted
    // ".withDriveRequestType(DriveRequestType.OpenLoopVoltage);"
    joystick
        .x()
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    setpointGen
                        .withVelocityX(MaxSpeed.times(-joystick.getLeftY()))
                        .withVelocityY(MaxSpeed.times(-joystick.getLeftX()))
                        .withRotationalRate(Constants.MaxAngularRate.times(-joystick.getRightX()))
                        .withOperatorForwardDirection(drivetrain.getOperatorForwardDirection())));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
  }

  public Command getAutonomousCommand() {
    return new SequentialCommandGroup(
        new MaxSpeed(motor), new RampSpeed(motor), new SetpointsAndSpeeds(motor));
  }
}
