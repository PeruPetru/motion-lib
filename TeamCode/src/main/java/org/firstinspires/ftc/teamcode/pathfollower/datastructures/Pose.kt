package org.firstinspires.ftc.teamcode.pathfollower.datastructures

class Pose(@JvmField var x: Double = 0.0,
           @JvmField var y: Double = 0.0,
           @JvmField var heading: Double = 0.0) {

    operator fun plus(pose: Pose): Pose = Pose(x + pose.x, y + pose.y, heading + pose.heading)

    operator fun times(scalar: Double): Pose = Pose(x * scalar, y * scalar, heading * scalar)
}