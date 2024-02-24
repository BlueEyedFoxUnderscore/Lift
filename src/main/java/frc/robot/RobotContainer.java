package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {

  //// private final CommandXboxController m_driverController =
  ////    new CommandXboxController(OperatorConstants.kDriverControllerPort);
  public static class SettableDoubleSupplier implements DoubleSupplier{
    double d;

    @Override
    public double getAsDouble() {
      // TODO Auto-generated method stub
      return d;
    }

    public void setDouble(double d){
      this.d = d;
    }
  }

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
