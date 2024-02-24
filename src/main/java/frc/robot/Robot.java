
package frc.robot;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.DynamicLiftCommand;
import frc.robot.commands.ZeroLiftCommand;
import frc.robot.subsystems.LiftSubsystem;

public class Robot extends TimedRobot {

  public static DigitalInput SRNSwitchL = new DigitalInput(0);
  public static DigitalInput SRNSwitchR = new DigitalInput(1);
  public static CANSparkMax lift_lMotor = new CANSparkMax(LiftSubsystem._c.lID, MotorType.kBrushless),
      lift_rMotor = new CANSparkMax(LiftSubsystem._c.rID, MotorType.kBrushless);
  private SparkPIDController m_pidController;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  private Command m_autonomousCommand;
  static LiftSubsystem lift = new LiftSubsystem();
  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    m_pidController = lift_lMotor.getPIDController();
    lift_lMotor.getEncoder();

    putConstants();
  }

  @Override
  public void robotPeriodic() {
    setConstants();
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void autonomousInit() {
    System.out.println("autonomousInit: Reached init");

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
    setConstants();
  }

  @Override
  public void teleopInit() {
    lift.setVelo_pair(0);

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    SmartDashboard.putNumber("Max Velocity", 5000);
    SmartDashboard.putNumber("Acceleration", 20000);
    SmartDashboard.putBoolean("Zero Lift", false);
  }

  @Override
  public void teleopPeriodic() {
    SmartDashboard.putBoolean("Stop Right Now", SRNSwitchL.get());
    setConstants();
    if (SmartDashboard.getBoolean("Zero Lift", false)) {
      (new ZeroLiftCommand(lift)).schedule();
      SmartDashboard.putBoolean("Zero Lift", false);
    }
    new DynamicLiftCommand(lift, new DoubleSupplier() { 
      
      @Override 
      public double getAsDouble() {return get() == idk()? put()? get(): idk(): get();}

      private double get(){return SmartDashboard.getNumber("WANTED POSITION", Double.NaN);}

      private boolean put(){return SmartDashboard.putNumber("WANTED POSITION", (lift.getEncoderL().getPosition() * LiftSubsystem._c.lRatio + lift.getEncoderR().getPosition())/2);}

      private double idk(){return Double.NaN;}
    
    }, 20000, 0, 5000).schedule();
  }

  @Override
  public void testInit() {

    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void simulationInit() {
  }

  @Override
  public void simulationPeriodic() {
  }

  private void setConstants() {

    SmartDashboard.putNumber("Position", lift.getEncoderL().getPosition());
    SmartDashboard.putNumber("Velocity", lift.getEncoderL().getVelocity());

    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);

    if ((p != kP)) {
      m_pidController.setP(p);
      kP = p;
    }
    if ((i != kI)) {
      m_pidController.setI(i);
      kI = i;
    }
    if ((d != kD)) {
      m_pidController.setD(d);
      kD = d;
    }
    if ((iz != kIz)) {
      m_pidController.setIZone(iz);
      kIz = iz;
    }
    if ((ff != kFF)) {
      m_pidController.setFF(ff);
      kFF = ff;
    }
    if ((max != kMaxOutput) || (min != kMinOutput)) {
      m_pidController.setOutputRange(min, max);
      kMinOutput = min;
      kMaxOutput = max;
    }
  }

  public void putConstants(){
    kP = m_pidController.getP();
    kI = m_pidController.getI();
    kD = m_pidController.getD();
    kIz = m_pidController.getIZone();
    kFF = m_pidController.getFF();
    kP = 6e-5;
    kI = 0.000001;
    kD = 0;
    kIz = 0;
    kFF = 0.000015;

    kMaxOutput = 1;
    kMinOutput = -1;

    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setIZone(kIz);
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);

    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
    SmartDashboard.putNumber("Set Rotations", 0);}
}
