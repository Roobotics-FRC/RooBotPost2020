@file:JvmName("Utils")
package frc.team4373.robot

import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min

fun constrainPercentOutput(output: Double): Double {
    return max(-1.0, min(output, 1.0))
}

fun isZero(n: Double): Boolean {
    return abs(n) < RobotMap.FP_EQUALITY_THRESHOLD
}