package org.firstinspires.ftc.teamcode.commandbase

import com.acmerobotics.dashboard.telemetry.TelemetryPacket

//Simple kotlin commandbase system, run method returns true if the command has finished

interface Command {
    fun run(packet: TelemetryPacket): Boolean
}