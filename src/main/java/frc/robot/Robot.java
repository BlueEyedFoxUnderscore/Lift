
package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.DriveLiftByJoystickCommand;
import frc.robot.commands.ZeroLiftCommand;
import frc.robot.subsystems.LiftSubsystem;

public class Robot extends TimedRobot {

  public class _c {
  }

  private static Joystick liftJoystick = new Joystick(0);
  public static CANSparkMax 
    lift_lMotor = new CANSparkMax(LiftSubsystem._c.ID_L, MotorType.kBrushless),
    lift_rMotor = new CANSparkMax(LiftSubsystem._c.ID_R, MotorType.kBrushless);
  private SparkPIDController l_pidController, r_pidController;
  public double 
    kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput,
    kP_r, kI_r, kD_r, kIz_r, kFF_r, kMaxOutput_r, kMinOutput_r;
  private Command m_autonomousCommand;
  private static Servo 
    RRL = new Servo(LiftSubsystem._c.RR_ID_L),
    RRR = new Servo(LiftSubsystem._c.RR_ID_R);
  static LiftSubsystem 
    liftL = new LiftSubsystem(lift_lMotor, RRL, LiftSubsystem._c.REVERSED_L, LiftSubsystem._c.LOCK_REVERSED_L, "left"),
    liftR = new LiftSubsystem(lift_rMotor, RRR, LiftSubsystem._c.REVERSED_R, LiftSubsystem._c.LOCK_REVERSED_R, "right");
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

    if(m_autonomousCommand != null) {
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

    new ZeroLiftCommand(liftL).schedule();
    new ZeroLiftCommand(liftR).schedule();


    if(m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  boolean liftLocked_l = false;
  boolean liftLocked_r = false;


  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {

    SmartDashboard.putBoolean("Zero Lift", false);
    SmartDashboard.putBoolean("Start move", false);
    SmartDashboard.putBoolean("Lock Lift Left", false);
    SmartDashboard.putBoolean("Lock Lift Right", false);
    SmartDashboard.putBoolean("Disable Dynamic", false);

    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
    SmartDashboard.putNumber("Position (Left)", liftL.getEncoder().getPosition());
    SmartDashboard.putNumber("Velocity (Left)", liftL.getEncoder().getVelocity());

    SmartDashboard.putNumber("Position (Right)", liftR.getEncoder().getPosition());
    SmartDashboard.putNumber("Velocity (Right)", liftR.getEncoder().getVelocity());

    if(SmartDashboard.getBoolean("Disable Dynamic", false)){
      liftL.setLiftPeriodicDisabled(true);
      liftR.setLiftPeriodicDisabled(true);
    }
    else {
      liftL.setLiftPeriodicDisabled(false);
      liftR.setLiftPeriodicDisabled(false);
      
    }

    if(SmartDashboard.getBoolean("Lock Lift Left", false)) liftL.lock(); else liftL.unlock();

    if(SmartDashboard.getBoolean("Lock Lift Right", false)) liftR.lock(); else liftR.unlock();

    if(SmartDashboard.getBoolean("Start Dynamic", false)){
      new DriveLiftByJoystickCommand(this::getUpdatedRoll, this::getUpdatedHeight, liftL, liftR).schedule();
    }
    SmartDashboard.putBoolean("Start Dynamic", false);

    //// setConstants_l();
    //// setConstants_r();

    if(SmartDashboard.getBoolean("Zero Lift", false)) {
      (new ZeroLiftCommand(liftL)).schedule();
      (new ZeroLiftCommand(liftR)).schedule();
      SmartDashboard.putBoolean("Zero Lift", false);
    }
    else SmartDashboard.putBoolean("Zero Lift", false);
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


    if((p != kP)) {
      l_pidController.setP(p);
      kP = p;
    }
    if((i != kI)) {
      l_pidController.setI(i);
      kI = i;
    }
    if((d != kD)) {
      l_pidController.setD(d);
      kD = d;
    }
    if((iz != kIz)) {
      l_pidController.setIZone(iz);
      kIz = iz;
    }
    if((ff != kFF)) {
      l_pidController.setFF(ff);
      kFF = ff;
    }
    if((max != kMaxOutput) || (min != kMinOutput)) {
      //// l_pidController.setOutputRange(min, max);
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
    double iz = SmartDashboard.getNumber("I Zone", 0.0000005);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 1);
    double min = SmartDashboard.getNumber("Min Output", -1);

    if((p != kP_r)) {
      r_pidController.setP(p);
      kP_r = p;
    }
    if((i != kI_r)) {
      r_pidController.setI(i);
      kI_r = i;
    }
    if((d != kD_r)) {
      r_pidController.setD(d);
      kD_r = d;
    }
    if((iz != kIz_r)) {
      r_pidController.setIZone(iz);
      kIz_r = iz;
    }
    if((ff != kFF_r)) {
      r_pidController.setFF(ff);
      kFF_r = ff;
    }
    if((max != kMaxOutput_r) || (min != kMinOutput_r)) {
      //// r_pidController.setOutputRange(min, max);
      kMinOutput_r = min;
      kMaxOutput_r = max;
    }
  }

  public void putConstants(){
    /* 
    //kP_l = l_pidController.getP();
    //kI_l = l_pidController.getI();
    //kD_l = l_pidController.getD();
    //kIz_l = l_pidController.getIZone();
    //kFF_l = l_pidController.getFF();
    */
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

  private double liftTiltPrevious = 0, liftTiltChange = 0, heightPrevious = 0, heightChange = 0;

  public static double clamp(double val, double min, double max) {
    return Math.max(min, Math.min(max, val));
  }

  private double getUpdatedRoll(){
    if(liftJoystick.getRawButton(1)) {
      liftTiltChange = liftJoystick.getX() * 50.0;
    }
    else {
      liftTiltPrevious=clamp(liftTiltPrevious + liftTiltChange, -155.0, 155.0);
      liftTiltChange = 0;
    }
    return clamp(liftTiltPrevious + liftTiltChange, -155.0, 155.0);
  }
  private double getUpdatedHeight(){
    if(liftJoystick.getRawButton(1)) {
       heightChange = liftJoystick.getY() * -155.0;
    }
    else{
      heightPrevious = clamp(heightPrevious + heightChange, 0.0, 155.0);
      heightChange = 0;
    }
    return clamp(heightPrevious + heightChange, 0.0, 155.0);
  }
}
