// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ArmIO {
  @AutoLog
  public static class ArmIOInputs {
    public Angle motorAngle = Degrees.of(0);
    public Voltage appliedVoltage = Volts.of(0.0);
    public Current rightStatorCurrent = Amps.of(0);
    public Current rightSupplyCurrent = Amps.of(0);
    public AngularVelocity motorVelocity = DegreesPerSecond.of(0);
  }

  public default void updateInputs(ArmIOInputs inputs) {}

  public default void runWristPercent(double percent) {}

  public default void runWristPosition(double angle) {}

  public default void runClawPercent(double percent) {}

  public default void runClawPosition(double position) {}
}
