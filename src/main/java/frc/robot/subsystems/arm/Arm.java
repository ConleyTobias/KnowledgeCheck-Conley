// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs;

  /** Creates a new TalonFX. */
  public Arm(ArmIO io) {
    this.io = io;
    this.inputs = new ArmIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("TalonFx", inputs);
  }

  public void runWristPercent(double percent) {
    io.runWristPercent(percent);
  }

  public void runWristPosition(double angle) {
    io.runWristPosition(angle);
  }

  public void runClawPercent(double percent) {
    io.runClawPercent(percent);
  }

  public void runClawPosition(double position) {
    io.runClawPosition(position);
  }
}
