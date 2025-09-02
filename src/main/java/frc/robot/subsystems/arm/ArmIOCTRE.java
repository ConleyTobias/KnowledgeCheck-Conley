package frc.robot.subsystems.arm;

public class ArmIOCTRE implements ArmIO {
  public ArmIOCTRE() {}

  @Override
  public void updateInputs(ArmIOInputs inputs) {}

  public void runWristPercent(double percent) {}

  public void runWristPosition(double angle) {}

  public void runClawPercent(double percent) {}

  public void runClawPosition(double position) {}
}
