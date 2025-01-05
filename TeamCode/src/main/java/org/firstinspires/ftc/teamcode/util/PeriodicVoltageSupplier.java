package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.lynx.LynxVoltageSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.function.Supplier;

public class PeriodicVoltageSupplier implements Supplier<Double> {

    private double lastKnownVoltage;
    private double lastReadTime = 0;
    private final LynxVoltageSensor lynxVoltageSensor;
    private final double SECONDS_BETWEEN_READS;
    private final SimpleLogManager logManager;


    public PeriodicVoltageSupplier(HardwareMap hardwareMap, double secondsBetweenReads){
        this.lynxVoltageSensor = hardwareMap.getAll(LynxVoltageSensor.class).iterator().next();
        this.SECONDS_BETWEEN_READS = secondsBetweenReads;
        this.logManager = SimpleLogManager.getInstance();
    }

    public PeriodicVoltageSupplier(HardwareMap hardwareMap){
        this(hardwareMap, 0.5);
    }

    @Override
    public Double get() {
        if(System.nanoTime()/1e9 > lastReadTime + SECONDS_BETWEEN_READS){
            lastReadTime = System.nanoTime()/1e9;
            lastKnownVoltage = lynxVoltageSensor.getVoltage();
            logManager.addLog(SimpleLogManager.Type.INFORMATION, "voltage updated", String.valueOf(lastKnownVoltage));
        }
        return lastKnownVoltage;
    }
}
