package frc.robot;

import frc.robot.commands.LiftCommand;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {

  //// private final CommandXboxController m_driverController =
  ////    new CommandXboxController(OperatorConstants.kDriverControllerPort);
  public static class SettableDoubleSupplier implements DoubleSupplier{
    double d;

    public SettableDoubleSupplier(double d){
      this.d = d;
    }

    @Override
    public double getAsDouble() {
      // TODO Auto-generated method stub
      return d;
    }

    public void setDouble(double d){
      this.d = d;
    }
  }

  static LiftCommand c = new LiftCommand(Robot.lift, new SettableDoubleSupplier(0));

  public static SettableDoubleSupplier liftSupplierThingy = new SettableDoubleSupplier(0);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
  }

  public Command getAutonomousCommand() {
    return c;
  }
}
