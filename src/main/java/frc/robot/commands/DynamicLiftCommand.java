package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.LiftSubsystem;

public class DynamicLiftCommand extends Command {
    private LiftSubsystem liftL;
    private final DoubleSupplier xFin;

    private double xNow, a, vNext, vNow = 0;

    private final double vMax;
    public DynamicLiftCommand(LiftSubsystem liftL, DoubleSupplier xFinal, double accel, double vNow, double vMax) {
        this.liftL = liftL;
        this.xFin = xFinal;
        this.vNow = vNow;
        this.vMax = vMax;
        this.a = accel;
    }

    @Override
    public void initialize() {
        addRequirements(liftL);
    }

    @Override
    public void execute() {
        cycle(liftL);
    }

    public void cycle(LiftSubsystem lift) {
        xNow = lift.getPosition();

        double xErr = xFin.getAsDouble() - xNow;

        double t0v = Math.sqrt(2 * Math.abs(xErr) / a);

        double vBack = t0v * a * Math.signum(xErr);

        vNext = vNow + a * Math.signum(vBack - vNow);

        if (vBack > 0)
            vNext = Math.min(vBack, Math.min(vNext, vMax));
        else
            vNext = Math.max(vBack, Math.min(vNext, vMax));

        if (Robot.SRNSwitchL.get())
            vNext = 0;

        vNow = vNext;

        lift.setVelo(vNext);
    }

    public boolean isFinished(){
        if(SmartDashboard.getBoolean("Stop Dynamo", false)){
            SmartDashboard.putBoolean("Stop Dynamo", false);
            return true;
        }
        return false;
    }
}