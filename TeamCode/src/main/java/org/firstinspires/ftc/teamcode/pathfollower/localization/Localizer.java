package org.firstinspires.ftc.teamcode.pathfollower.localization;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.teamcode.pathfollower.datastructures.Pose;

public interface Localizer {
    void setPose(Pose pose);
    Pose getPose();
    void update(TelemetryPacket telemetryPacket);
}
