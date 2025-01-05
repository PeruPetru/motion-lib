package org.firstinspires.ftc.teamcode.util;

public class PDSGController {

    public enum FeedforwardType{
        LINEAR,
        SINUSOIDAL;
    }

    public PDSGCoefficients PDSG_COEFFICIENTS;
    public FeedforwardType FEEDFORWARD_TYPE = FeedforwardType.LINEAR;
    public double lowerLimit;
    public double upperLimit;


    private double tolerance = 1;
    private double target = 0;
    private double lastValue = 0;
    private double lastUpdateTime;

    public PDSGController(PDSGCoefficients pdsgCoefficients){
        this(pdsgCoefficients, -1e9, 1e9);
    }

    public PDSGController(PDSGCoefficients pdsgCoefficients, double lowerLimit, double upperLimit){
        this.PDSG_COEFFICIENTS = pdsgCoefficients;
        this.lowerLimit = lowerLimit;
        this.upperLimit = upperLimit;
    }

    public PDSGController(double p, double d, double s, double minG, double maxG, double lowerLimit, double upperLimit) {
        this(new PDSGCoefficients(p, d, s, minG, maxG), lowerLimit, upperLimit);
    }

    public PDSGController(double p, double d, double s, double minG, double maxG){
        this(new PDSGCoefficients(p, d, s, minG, maxG));
    }


    public void setPDSG_COEFFICIENTS(double p, double d, double s, double minG, double maxG) {
        this.PDSG_COEFFICIENTS = new PDSGCoefficients(p, d, s, minG, maxG);
    }

    public void setPDSG_COEFFICIENTS(PDSGCoefficients pdsgCoefficients) {
        this.PDSG_COEFFICIENTS = pdsgCoefficients;
    }

    public PDSGCoefficients getPID_COEFFICIENTS() {
        return PDSG_COEFFICIENTS;
    }

    public PDSGController setTarget(double target) {
        this.target = target;
        return this;
    }

    public double getTarget() {
        return target;
    }

    public void setTolerance(double tolerance) {
        this.tolerance = tolerance;
    }

    public double getTolerance() {
        return tolerance;
    }

    public boolean atTarget(){
        return Math.abs(target - lastValue) <= tolerance;
    }

    public double update(double currentValue){
        double error = target-currentValue;
        double deltaTime = lastUpdateTime - System.nanoTime() / 1e9;
        double deltaVelocity = (lastValue - currentValue) / deltaTime;

        lastValue = currentValue;
        lastUpdateTime = System.nanoTime() / 1e9;

        double proportionalPower = error * PDSG_COEFFICIENTS.p;
        double derivativePower = deltaVelocity * PDSG_COEFFICIENTS.d;
        double staticPower = (target != 0) ? Math.signum(error) : 0;
        double feedforwardPower = (FEEDFORWARD_TYPE == FeedforwardType.LINEAR) ?
                PDSG_COEFFICIENTS.minG + (PDSG_COEFFICIENTS.maxG - PDSG_COEFFICIENTS.minG) * (currentValue / upperLimit):
                PDSG_COEFFICIENTS.maxG * Math.cos(currentValue);

        return proportionalPower - derivativePower + staticPower + feedforwardPower;
    }
}
