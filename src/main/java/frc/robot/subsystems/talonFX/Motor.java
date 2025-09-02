// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.talonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Motor extends SubsystemBase {
  private final MotorIO io;
  private final MotorIOInputsAutoLogged inputs;

  /** Creates a new TalonFX. */
  public Motor(MotorIO io) {
    this.io = io;
    this.inputs = new MotorIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("TalonFx", inputs);
  }

  public void runPercent(double percent) {
    io.runPercent(percent);
  }

  public void runVoltage(double voltage) {
    io.runVoltage(voltage);
  }

  public void runPosition(double angle) {
    io.runPosition(angle);
  }

  public void runVelocity(double velocity) {
    io.runVelocity(velocity);
  }
}
