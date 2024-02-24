package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class LiftSubsystem extends SubsystemBase {
    CANSparkMax l = Robot.lift_lMotor;
    CANSparkMax r = Robot.lift_rMotor;
    RelativeEncoder lEncoder = l.getEncoder();
    RelativeEncoder rEncoder = r.getEncoder();
    SparkPIDController lPid = l.getPIDController();
    SparkPIDController rPid = r.getPIDController();

    public static class _c {
        public static final double lRatio = 20;
        public static final double rRatio = 70;
        public static final int lID = 12;
        public static final int rID = 13;
    }

    public void setVelo_pair(double speed) {
        lPid.setReference(speed * _c.lRatio, CANSparkMax.ControlType.kVelocity);
        rPid.setReference(speed * _c.rRatio, CANSparkMax.ControlType.kVelocity);
        // System.out.println("setVelo: Set speed to " + speed);
    }

    public void setPos_pair(double pos) {
        lPid.setReference(pos * _c.lRatio, CANSparkMax.ControlType.kPosition);
        rPid.setReference(pos * _c.rRatio, CANSparkMax.ControlType.kPosition);
        // System.out.println("setSpeed: Did not set position to" + pos);
    }

    public void setVelo_l(double speed) {
        lPid.setReference(speed * _c.lRatio, CANSparkMax.ControlType.kVelocity);
    }

    public void setVelo_r(double speed) {
        rPid.setReference(speed * _c.rRatio, CANSparkMax.ControlType.kVelocity);
    }

    public RelativeEncoder getEncoderL() {
        return lEncoder;
    }

    public RelativeEncoder getEncoderR() {
        return rEncoder;
    }

    public double getPositionL() {
        return lEncoder.getPosition();
    }

    public double getPositionR() {
        return rEncoder.getPosition();
    }
}
