package org.firstinspires.ftc.teamcode.robot.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.robot.Robot;

public class SplineTest extends OpMode {

    FtcDashboard ftcDashboard;
    Robot robot;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        ftcDashboard = FtcDashboard.getInstance();
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start(){

    }

    @Override
    public void loop() {
        TelemetryPacket telemetryPacket = new TelemetryPacket();
        robot.update();
    }
}
