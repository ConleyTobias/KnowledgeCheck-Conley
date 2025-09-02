package frc.robot.subsystems.talonFX;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class MotorIOSim extends MotorIOCTRE {

  private final TalonFX talonfx = new TalonFX(1); // makes the real motor
  private final TalonFX follower = new TalonFX(2);

  private final DCMotorSim sim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 0.0001, 6),
          DCMotor.getKrakenX60Foc(1)); // creates sim motor

  TalonFXConfiguration config = new TalonFXConfiguration(); // makes real motor config

  // PID
  private double kP = 1250; // PID values
  private double kI = 0;
  private double kD = 10; // PID values
  ProfiledPIDController pid =
      new ProfiledPIDController(
          kP, kI, kD, new TrapezoidProfile.Constraints(1000, 1000)); // Creates a new PID controller

  public MotorIOSim() {
    // Makes and applys config
    config.withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimitEnable(false));
    config.Slot0.withKP(1250).withKI(0).withKD(10).withKS(0).withKV(0).withKA(0);
    config.MotionMagic.withMotionMagicCruiseVelocity(1000)
        .withMotionMagicAcceleration(1000)
        .withMotionMagicJerk(0);

    // Log PID
    SmartDashboard.putNumber("kP", config.Slot0.kP);
    SmartDashboard.putNumber("kI", config.Slot0.kI);

    talonfx.getConfigurator().apply(config);
    follower.getConfigurator().apply(config);
    follower.setControl(new Follower(1, true)); // makes the follower follow the real motor
  }

  public void updateInputs(MotorIOInputs inputs) {
    var talonFXSim = talonfx.getSimState(); // gets current simulation
    var motorVoltage = talonFXSim.getMotorVoltageMeasure(); // gets motor voltage

    talonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage()); // updates battery percentage
    sim.setInputVoltage(motorVoltage.in(Volts)); // updates motor voltage

    sim.update(0.020); // may be out of order (hopefully not)

    talonFXSim.setRawRotorPosition(sim.getAngularPosition().times(1)); // sets sim to real postion
    talonFXSim.setRotorVelocity(sim.getAngularVelocity().times(1)); // same as above

    // Update Inputs
    inputs.appliedVoltage = motorVoltage;
    inputs.motorAngle = sim.getAngularPosition();
    inputs.motorVelocity = sim.getAngularVelocity();
    inputs.rightStatorCurrent = talonfx.getStatorCurrent().getValue();
    inputs.rightSupplyCurrent = talonFXSim.getSupplyCurrentMeasure();

    // Update SmartDashboard
    SmartDashboard.putNumber("VOLT", inputs.appliedVoltage.in(Volts));
    SmartDashboard.putNumber("ANGLE", inputs.motorAngle.in(Degrees));
    SmartDashboard.putNumber("VELOCITY", inputs.motorVelocity.in(RPM));
    SmartDashboard.putNumber("STATOR", inputs.rightStatorCurrent.in(Amps));
    SmartDashboard.putNumber("SUPPLY", inputs.rightSupplyCurrent.in(Amps));
    SmartDashboard.putNumber("SETPOINT", talonfx.getClosedLoopReference().getValueAsDouble() * 360);

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
  public void runPercent(double percent) {
    talonfx.setVoltage(percent);
  }

  @Override
  public void runVoltage(double voltage) {
    talonfx.setVoltage(voltage);
  }

  public double getPosition() {
    return talonfx.getRotorPosition().getValueAsDouble(); // returns the position of the motor
  }

  public double getVelocity() {
    return talonfx.getVelocity().getValueAsDouble(); // returns the velocity of the motor
  }

  @Override
  /*Rotations */
  public void runPosition(double position) {
    talonfx.set(pid.calculate(getPosition(), position)); // runs the motor to the position
  }

  @Override
  public void runVelocity(double velocity) {
    talonfx.set(pid.calculate(getVelocity(), velocity)); // sets the velocity reference
  }
}
