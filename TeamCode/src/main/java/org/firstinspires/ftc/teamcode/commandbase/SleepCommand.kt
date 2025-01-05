package org.firstinspires.ftc.teamcode.commandbase

import com.acmerobotics.dashboard.telemetry.TelemetryPacket

class SleepCommand(val deltaTime: Double): Command {
    var startTime: Double = -1.0

    override fun run(packet: TelemetryPacket): Boolean {
        //start waiting from the first run
        if(startTime == -1.0){
            startTime = System.nanoTime() / 1e9
        }
        return startTime + deltaTime <= System.nanoTime() / 1e9
    }
}