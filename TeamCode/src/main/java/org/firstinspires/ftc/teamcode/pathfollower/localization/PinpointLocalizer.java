package org.firstinspires.ftc.teamcode.pathfollower.localization;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pathfollower.datastructures.Pose;
import org.firstinspires.ftc.teamcode.util.GoBildaPinpointDriver;

public class PinpointLocalizer implements Localizer{
    GoBildaPinpointDriver pinpoint;

    private static final double PINPOINT_X_OFFSET = 0;
    private static final double PINPOINT_Y_OFFSET = 0;
    private static final double TICKS_PER_CM = 0;

    public PinpointLocalizer(HardwareMap hardwareMap){
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setPosition(new Pose(0, 0, 0));
        pinpoint.setOffsets(PINPOINT_X_OFFSET, PINPOINT_Y_OFFSET);
        pinpoint.setEncoderResolution(TICKS_PER_CM * 10.0);
    }

    @Override
    public void setPose(Pose pose) {
        pinpoint.setPosition(pose);
    }

    @Override
    public Pose getPose() {
        return pinpoint.getPosition();
    }

    @Override
    public void update(TelemetryPacket telemetryPacket) {
        pinpoint.update();
    }
}
