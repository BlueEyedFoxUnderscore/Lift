package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class LiftSubsystem extends SubsystemBase {
    CANSparkMax lift = Robot.m_motor;
    RelativeEncoder liftEncoder = lift.getEncoder();
    SparkPIDController pidController = lift.getPIDController();

    public void setVelo(double speed) {
        pidController.setReference(speed, CANSparkMax.ControlType.kVelocity);
        System.out.println("setVelo: Set speed to " + speed);
    }

    public void setPos(double pos){
        //pidController.setReference(pos, CANSparkMax.ControlType.kPosition);
        System.out.println("setSpeed: Did not set position to" + pos);
    }

    public RelativeEncoder getEncoder(){
        return liftEncoder;
    }
}
