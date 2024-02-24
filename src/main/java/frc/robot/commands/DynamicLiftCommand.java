package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.LiftSubsystem;

public class DynamicLiftCommand extends Command {
    private LiftSubsystem lift = new LiftSubsystem();
    private boolean isFinished;
    private final DoubleSupplier xFin;

    private double xNow, a, vNext, vNow = 0;

    private final double vMax;
    public DynamicLiftCommand(LiftSubsystem lift, DoubleSupplier xFinal, double accel, double vNow, double vMax) {
        this.lift = lift;
        this.xFin = xFinal;
        this.vNow = vNow;
        this.vMax = vMax;
        this.a = accel;
    }

    @Override
    public void initialize() {
        addRequirements(lift);
    }

    @Override
    public void execute() {
        cycleL();
        cycleR();
    }

    @Override
    public boolean isFinished() {
        return this.isFinished;
    }

    public void cycleL() {
        xNow = lift.getPositionL();

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

        lift.setVelo_pair(vNext);
    }

    public void cycleR() {
        xNow = lift.getPositionR();

        double xErr = xFin.getAsDouble() - xNow;

        double t0v = Math.sqrt(2 * Math.abs(xErr) / a);

        double vBack = t0v * a * Math.signum(xErr);

        vNext = vNow + a * Math.signum(vBack - vNow);

        if (vBack > 0)
            vNext = Math.min(vBack, Math.min(vNext, vMax));
        else
            vNext = Math.max(vBack, Math.min(vNext, vMax));

        if (Robot.SRNSwitchR.get())
            vNext = 0;

        vNow = vNext;

        lift.setVelo_pair(vNext);
    }
}