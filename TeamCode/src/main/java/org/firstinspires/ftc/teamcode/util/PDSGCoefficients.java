package org.firstinspires.ftc.teamcode.util;

public class PDSGCoefficients
{
    public double p;
    public double d;
    public double s;
    public double maxG;
    public double minG;

    public PDSGCoefficients()
    {
        this(0,0,0,0,0);
    }

    public PDSGCoefficients(double p, double d, double s, double minG, double maxG)
    {
        this.p = p;
        this.d = d;
        this.s = s;
        this.minG = minG;
        this.maxG = maxG;
    }
}
