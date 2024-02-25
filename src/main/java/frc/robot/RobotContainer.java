package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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
    new Trigger(Robot.SRNSwitchL::get).onTrue(new Command(){
      @Override
      public void initialize(){
        Robot.liftL.setVelo(0);
      }
      @Override
      public boolean isFinished(){
        return true;
      }
    });
    new Trigger(Robot.SRNSwitchR::get).onTrue(new Command(){
      @Override
      public void initialize(){
        Robot.liftR.setVelo(0);
      }
      @Override
      public boolean isFinished(){
        return true;
      }
    });
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
