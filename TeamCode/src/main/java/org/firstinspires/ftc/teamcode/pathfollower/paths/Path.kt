package org.firstinspires.ftc.teamcode.pathfollower.paths

import org.firstinspires.ftc.teamcode.pathfollower.datastructures.Pose

interface Path {
    fun get(t: Double): Pose?
    fun getClosestPointParameter(point: Pose?): Double
    fun atEnd(point: Pose?): Boolean
}
