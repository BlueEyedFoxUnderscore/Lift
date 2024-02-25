
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
import frc.robot.commands.MoveLiftCommand;
import frc.robot.commands.ZeroLiftCommand;
import frc.robot.subsystems.LiftSubsystem;

public class Robot extends TimedRobot {

  public static DigitalInput 
    SRNSwitchL = new DigitalInput(LiftSubsystem._c.SRN_ID_L), 
    SRNSwitchR = new DigitalInput(LiftSubsystem._c.SRN_ID_R);
  public static CANSparkMax 
    lift_lMotor = new CANSparkMax(LiftSubsystem._c.ID_L, MotorType.kBrushless),
    lift_rMotor = new CANSparkMax(LiftSubsystem._c.ID_R, MotorType.kBrushless);
  private SparkPIDController l_pidController, r_pidController;
  public double 
    kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput,
    kP_r, kI_r, kD_r, kIz_r, kFF_r, kMaxOutput_r, kMinOutput_r;
  private Command m_autonomousCommand;
  static LiftSubsystem 
    liftL = new LiftSubsystem(lift_lMotor, SRNSwitchL, LiftSubsystem._c.RATIO_L),
    liftR = new LiftSubsystem(lift_rMotor, SRNSwitchR, LiftSubsystem._c.RATIO_R);
  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    l_pidController = lift_lMotor.getPIDController();
    r_pidController = lift_rMotor.getPIDController();
    putConstants();
  }

  @Override
  public void robotPeriodic() {
    //putConstants();
    setConstants_l();
    setConstants_r();
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    //putConstants();
    setConstants_l();
    setConstants_r();
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
    setConstants_l();
    setConstants_r();
  }

  @Override
  public void teleopInit() {
    liftL.setVelo(0);
    liftR.setVelo(0);

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    SmartDashboard.putBoolean("Zero Lift", false);
  
    SmartDashboard.putNumber("Target Position", 0);
    SmartDashboard.putNumber("Max Velocity", 5000);
    SmartDashboard.putNumber("Acceleration", 20000);
    SmartDashboard.putBoolean("Start move", false);
  }

  @Override
  public void teleopPeriodic() {
    if(SmartDashboard.getBoolean("Start Dynamo", false))
      new DynamicLiftCommand(liftL, liftR, new DoubleSupplier() { 
        @Override 
        public double getAsDouble() {return get() == idk()? put()? get(): idk(): get();}

        private double get(){return SmartDashboard.getNumber("Wanted Position", Double.NaN);}

        private boolean put(){return SmartDashboard.putNumber("Wanted Position", (liftL.getEncoder().getPosition() * LiftSubsystem._c.RATIO_L + liftR.getEncoder().getPosition() * LiftSubsystem._c.RATIO_R)/2);}

        private double idk(){return Double.NaN;}
      
      }, 20000, 0, 5000).schedule();
    else SmartDashboard.putBoolean("Start Dynamo", false);


    SmartDashboard.putBoolean("SRN Left", SRNSwitchL.get());
    SmartDashboard.putBoolean("SRN Right", SRNSwitchR.get());
    setConstants_l();
    setConstants_r();
    if (SmartDashboard.getBoolean("Zero Lift", false)) {
      (new ZeroLiftCommand(liftL)).schedule();
      (new ZeroLiftCommand(liftR)).schedule();
      SmartDashboard.putBoolean("Zero Lift", false);
    }
  
    if(SmartDashboard.getBoolean("Start move", false)){
      SmartDashboard.putBoolean("Start move", false);
      (new MoveLiftCommand(
        liftL, 
        SmartDashboard.getNumber("Target Position", 0) * LiftSubsystem._c.RATIO_L, 
        SmartDashboard.getNumber("Max Velocity", 5000), 
        SmartDashboard.getNumber("Acceleration", 200000)
      )).schedule();
      (new MoveLiftCommand(
        liftR, 
        SmartDashboard.getNumber("Target Position", 0) * LiftSubsystem._c.RATIO_R, 
        SmartDashboard.getNumber("Max Velocity", 5000), 
        SmartDashboard.getNumber("Acceleration", 200000)
      )).schedule();
    }
    else SmartDashboard.putBoolean("Start move", false);
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

  private void setConstants_l() {

    SmartDashboard.putNumber("Position (Left)", liftL.getEncoder().getPosition());
    SmartDashboard.putNumber("Velocity (Left)", liftL.getEncoder().getVelocity());

    double p = SmartDashboard.getNumber("P Gain", .00006);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0.000001);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 1);
    double min = SmartDashboard.getNumber("Min Output", -1);


    if ((p != kP)) {
      l_pidController.setP(p);
      kP = p;
    }
    if ((i != kI)) {
      l_pidController.setI(i);
      kI = i;
    }
    if ((d != kD)) {
      l_pidController.setD(d);
      kD = d;
    }
    if ((iz != kIz)) {
      l_pidController.setIZone(iz);
      kIz = iz;
    }
    if ((ff != kFF)) {
      l_pidController.setFF(ff);
      kFF = ff;
    }
    if ((max != kMaxOutput) || (min != kMinOutput)) {
      l_pidController.setOutputRange(min, max);
      kMinOutput = min;
      kMaxOutput = max;
    }
  }

  private void setConstants_r() {
    SmartDashboard.putNumber("Position (Right)", liftR.getEncoder().getPosition());
    SmartDashboard.putNumber("Velocity (Right)", liftR.getEncoder().getVelocity());

    double p = SmartDashboard.getNumber("P Gain", .00006);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0.000001);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 1);
    double min = SmartDashboard.getNumber("Min Output", -1);

    if ((p != kP_r)) {
      r_pidController.setP(p);
      kP_r = p;
    }
    if ((i != kI_r)) {
      r_pidController.setI(i);
      kI_r = i;
    }
    if ((d != kD_r)) {
      r_pidController.setD(d);
      kD_r = d;
    }
    if ((iz != kIz_r)) {
      r_pidController.setIZone(iz);
      kIz_r = iz;
    }
    if ((ff != kFF_r)) {
      r_pidController.setFF(ff);
      kFF_r = ff;
    }
    if ((max != kMaxOutput_r) || (min != kMinOutput_r)) {
      r_pidController.setOutputRange(min, max);
      kMinOutput_r = min;
      kMaxOutput_r = max;
    }
  }

  public void putConstants(){
    //kP_l = l_pidController.getP();
    //kI_l = l_pidController.getI();
    //kD_l = l_pidController.getD();
    //kIz_l = l_pidController.getIZone();
    //kFF_l = l_pidController.getFF();
    kP = .00006;
    kI = 0.000001;
    kD = 0;
    kIz = 0;
    kFF = 0.000015;
    kMaxOutput = 1;
    kMinOutput = -1;

    l_pidController.setP(kP);
    l_pidController.setI(kI);
    l_pidController.setD(kD);
    l_pidController.setIZone(kIz);
    l_pidController.setFF(kFF);
    l_pidController.setOutputRange(kMinOutput, kMaxOutput);
    r_pidController.setP(kP);
    r_pidController.setI(kI);
    r_pidController.setD(kD);
    r_pidController.setIZone(kIz);
    r_pidController.setFF(kFF);
    r_pidController.setOutputRange(kMinOutput, kMaxOutput);

    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
    SmartDashboard.putNumber("Set Rotations", 0);}
}
