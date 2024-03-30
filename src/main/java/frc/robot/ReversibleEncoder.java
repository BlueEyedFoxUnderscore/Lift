package frc.robot;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;

public class ReversibleEncoder implements RelativeEncoder {

    final RelativeEncoder e;
    final double multiplier;

    public ReversibleEncoder(RelativeEncoder e, boolean reversed){
        this.e = e;
        multiplier = reversed ? -1: 1;
    }

    @Override
    public double getPosition() {
        return e.getPosition() * multiplier;
    }

    @Override
    public double getVelocity() {
        return e.getVelocity() * multiplier;
    }

    @Override
    public REVLibError setPosition(double position) {
        return e.setPosition(position * multiplier);
    }

    @Override
    public REVLibError setPositionConversionFactor(double factor) {
        return e.setPositionConversionFactor(factor);
    }

    @Override
    public REVLibError setVelocityConversionFactor(double factor) {
        return e.setVelocityConversionFactor(factor);
    }

    @Override
    public double getPositionConversionFactor() {
        return e.getPositionConversionFactor();
    }

    @Override
    public double getVelocityConversionFactor() {
        return e.getVelocityConversionFactor();
    }

    @Override
    public REVLibError setAverageDepth(int depth) {
        return e.setAverageDepth(depth);
    }

    @Override
    public int getAverageDepth() {
        return e.getAverageDepth();
    }

    @Override
    public REVLibError setMeasurementPeriod(int period_ms) {
        return e.setMeasurementPeriod(period_ms);
    }

    @Override
    public int getMeasurementPeriod() {
        return e.getMeasurementPeriod();
    }

    @Override
    public int getCountsPerRevolution() {
        return e.getCountsPerRevolution();
    }

    @Override
    public REVLibError setInverted(boolean inverted) {
        return e.setInverted(inverted);
    }

    @Override
    public boolean getInverted() {
        return e.getInverted();
    }
    
}
