package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmIOSim extends ArmIOCTRE {

  private final TalonFX wrist = new TalonFX(1); // makes the real motor
  private final TalonFX claw = new TalonFX(2);

  private final SingleJointedArmSim sim =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60(1),
          getWristPosition(),
          getWristPosition(),
          getWristPosition(),
          getWristPosition(),
          getWristPosition(),
          false,
          getWristPosition(),
          null);

  TalonFXConfiguration config = new TalonFXConfiguration(); // makes real motor config

  // PID
  private double kP = 1250; // PID values
  private double kI = 0;
  private double kD = 10; // PID values

  ProfiledPIDController pid =
      new ProfiledPIDController(
          kP, kI, kD, new TrapezoidProfile.Constraints(1000, 1000)); // Creates a new PID controller

  public ArmIOSim() {
    // Makes and applys config
    config.withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimitEnable(false));
    config.Slot0.withKP(1250).withKI(0).withKD(10).withKS(0).withKV(0).withKA(0);
    config.MotionMagic.withMotionMagicCruiseVelocity(1000)
        .withMotionMagicAcceleration(1000)
        .withMotionMagicJerk(0);

    // Log PID
    SmartDashboard.putNumber("kP", config.Slot0.kP);
    SmartDashboard.putNumber("kI", config.Slot0.kI);

    wrist.getConfigurator().apply(config);
    claw.getConfigurator().apply(config);
  }

  public void updateInputs(ArmIOInputs inputs) {
    var wristSim = wrist.getSimState(); // gets current simulation
    var clawSim = claw.getSimState();
    var motorVoltage = wristSim.getMotorVoltageMeasure(); // gets motor voltage

    wristSim.setSupplyVoltage(RobotController.getBatteryVoltage()); // updates battery percentage
    sim.setInputVoltage(motorVoltage.in(Volts)); // updates motor voltage

    sim.update(0.020); // may be out of order (hopefully not)

    wristSim.setRawRotorPosition(sim.getAngleRads()); // sets sim to real postion
    wristSim.setRotorVelocity(sim.getAngleRads()); // same as above

    // Update Inputs
    inputs.appliedVoltage = motorVoltage;
    inputs.motorAngle =
        Angle.ofRelativeUnits(
            Units.degreesToRadians(90) - (sim.getAngleRads() - (2 * Math.PI)), Radian);

    // Update SmartDashboard
    SmartDashboard.putNumber("Arm_VOLT", inputs.appliedVoltage.in(Volts));
    SmartDashboard.putNumber("Arm_ANGLE", inputs.motorAngle.in(Degrees));
    SmartDashboard.putNumber("Arm_VELOCITY", inputs.motorVelocity.in(RPM));
    SmartDashboard.putNumber("Arm_STATOR", inputs.rightStatorCurrent.in(Amps));
    SmartDashboard.putNumber("Arm_SUPPLY", inputs.rightSupplyCurrent.in(Amps));
    SmartDashboard.putNumber(
        "Arm_SETPOINT", wrist.getClosedLoopReference().getValueAsDouble() * 360);

    // If anything changed, update (updating method is configUpd())
    if (SmartDashboard.getNumber("kP", kP) != kP
        || SmartDashboard.getNumber("kI", config.Slot0.kI) != config.Slot0.kI
        || SmartDashboard.getNumber("kD", config.Slot0.kD) != config.Slot0.kD) {
      configUpd();
    }
  }

  private void configUpd() {
    // changes what was changed 🤯
    kP = SmartDashboard.getNumber("kP", 0);
    kI = SmartDashboard.getNumber("kI", 0);
    kD = SmartDashboard.getNumber("kD", 0);
  }

  @Override
  public void runWristPercent(double percent) {
    wrist.set(percent);
  }

  public double getWristPosition() {
    return wrist.getRotorPosition().getValueAsDouble(); // returns the position of the motor
  }

  @Override
  /*Rotations */
  public void runWristPosition(double position) {
    wrist.set(pid.calculate(getWristPosition(), position)); // runs the motor to the position
  }

  @Override
  public void runClawPercent(double percent) {
    claw.set(percent);
  }

  public double getClawPosition() {
    return claw.getRotorPosition().getValueAsDouble();
  }

  @Override
  public void runClawPosition(double position) {
    claw.set(pid.calculate(getClawPosition(), position));
  }
}
