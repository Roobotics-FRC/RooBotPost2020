package frc.team4373.robot;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.util.Units;

/**
 * A Javadoc template. TODO: Update ConversionUtils Javadoc.
 */
public class ConversionUtils {
    private ConversionUtils() {
    }

    public static double toEncoderUnits(Rotation2d rot) {
        return rot.getDegrees() * 4096.0 / 360.0;
    }

    public static double toPigeonUnits(Rotation2d rot) {
        return rot.getDegrees() * 8192.0 / 360.0;
    }

    public static Rotation2d fromEncoderUnits(double units) {
        return Rotation2d.fromDegrees(units * 360.0 / 4096.0);
    }

    public static Rotation2d fromPigeonUnits(double units) {
        return Rotation2d.fromDegrees(units * 360.0 / 8192.0);
    }

    public static double metersPerSecondToEncoderUnitsPerDecisecond(double mps) {
        return mps * Units.metersToInches(1.0) / RobotMap.ENCODER_UNITS_TO_INCHES / 10.0; /*seconds to deciseconds*/
    }

    public static double encoderUnitsPerDecisecondToMetersPerSecond(double epds) {
        return epds * 10.0 * RobotMap.ENCODER_UNITS_TO_INCHES * Units.inchesToMeters(1.0);
    }
}