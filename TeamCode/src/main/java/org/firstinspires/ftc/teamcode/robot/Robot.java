package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pathfollower.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.SimpleLogManager;
import org.firstinspires.ftc.teamcode.util.PeriodicVoltageSupplier;

import java.util.function.Supplier;


public class Robot {
    public SimpleLogManager logManager;
    public MecanumDrive mecanumDrive;
    public Supplier<Double> voltageSupplier;

    public Robot(HardwareMap hardwareMap){
        voltageSupplier = new PeriodicVoltageSupplier(hardwareMap);

        this.mecanumDrive = new MecanumDrive(hardwareMap, voltageSupplier);


        for(LynxModule lynxModule : hardwareMap.getAll(LynxModule.class))
            lynxModule.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
    }

    public void update(TelemetryPacket telemetryPacket){
        mecanumDrive.update(telemetryPacket);
    }
}
