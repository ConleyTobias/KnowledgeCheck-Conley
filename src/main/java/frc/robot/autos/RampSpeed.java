// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.talonFX.Motor;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RampSpeed extends Command {
  private final Motor motor;
  public final Timer timer = new Timer();

  /** Creates a new RampSpeed. */
  public RampSpeed(Motor motor) {
    this.motor = motor;
    addRequirements(motor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    motor.runPercent(timer.get() * 10);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    motor.runPercent(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > 10;
  }
}
