@file:JvmName("ConversionUtils")
package frc.team4373.robot

import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.util.Units


fun Rotation2d.toEncoderUnits(): Double {
    return this.degrees * 4096.0 / 360.0
}
fun Rotation2d.toPigeonUnits(): Double {
    return this.degrees * 8192.0 / 360.0
}
fun fromEncoderUnits(units: Double): Rotation2d {
    return Rotation2d.fromDegrees(units * 360.0 / 4096.0)
}
fun fromPigeonUnits(units: Double): Rotation2d {
    return Rotation2d.fromDegrees(units * 360.0 / 8192.0)
}
fun Double.metersPerSecondToEncoderUnitsPerDecisecond(): Double {
    return this * Units.metersToInches(1.0) / RobotMap.ENCODER_UNITS_TO_INCHES / 10.0 /*seconds to deciseconds*/
}
fun Double.encoderUnitsPerDecisecondToMetersPerSecond(): Double {
    return this * 10.0 * RobotMap.ENCODER_UNITS_TO_INCHES * Units.inchesToMeters(1.0)
}