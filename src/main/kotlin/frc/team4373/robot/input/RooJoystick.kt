package frc.team4373.robot.input

import edu.wpi.first.wpilibj.Joystick
import frc.team4373.robot.input.filters.DoubleTypeFilter
import java.util.concurrent.ConcurrentHashMap

/**
 * This class extends the WPILib Joystick class to add deadzone and filter functionality.
 */
class RooJoystick(port: Int) : Joystick(port) {
    /**
     * Constructs a configuration for an axis with the specified filter and deadzone.
     * @param filter the filter through which to process all axis values.
     * @param deadzone the threshold below which all values will be zeroed.
     */
    private data class AxisConfiguration(val filter: DoubleTypeFilter, val deadzone: Double)

    private var defaultFilter: DoubleTypeFilter? = null
    private var defaultDeadzone: Double = 0.0
    private val axisMap: MutableMap<Int, AxisConfiguration>

    /**
     * Constructs a joystick on the specified port.
     * @param port the port to which the joystick is connected.
     */
    init {
        // Joystick access can be multithreaded, so use a thread-safe type
        axisMap = ConcurrentHashMap()
    }

    /**
     * Constructs a joystick on the specified port with a default filter and deadzone.
     * These defaults can be overridden on a per-axis basis. All axes without specified
     * filters and deadzones will be processed using the default values.
     * @param port the port to which the joystick is connected.
     * @param defaultFilter the filter to apply by default to joystick axes.
     * @param defaultDeadzone the deadzone to apply by default to joystick axes.
     */
    constructor(port: Int, defaultFilter: DoubleTypeFilter?, defaultDeadzone: Double): this(port) {
        this.defaultFilter = defaultFilter
        this.defaultDeadzone = defaultDeadzone
    }

    /**
     * Configures the filter and deadzone for the specified axis.
     * @param axis the axis to configure.
     * @param filter the filter to apply to the axis.
     * @param deadzone the axis' deadzone (all values <= this magnitude will be zeroed).
     */
    fun configureAxis(axis: Int, filter: DoubleTypeFilter, deadzone: Double) {
        axisMap[axis] = AxisConfiguration(filter, deadzone)
    }

    /**
     * Ignores input if it is within the deadzone (if it is negligible).
     * @param input the input value to be checked.
     * @param deadzone the deadzone value against which to check.
     * @return the input value if it is large enough, or 0 if it is negligible.
     */
    private fun applyDeadzone(input: Double, deadzone: Double): Double {
        return if (Math.abs(input) <= deadzone) 0.0 else input
    }

    /**
     * Returns the filtered and deadzoned value of a joystick axis.
     * @param axis the axis to read from.
     * @return the processed axis value.
     */
    fun getAxis(axis: Int): Double {
        val rawAxis = getRawAxis(axis)
        val config = axisMap[axis]
        if (config == null) {
            val df = defaultFilter
            if (df != null) {
                return applyDeadzone(df.applyFilter(rawAxis), defaultDeadzone)
            }
            return applyDeadzone(rawAxis, defaultDeadzone)
        }
        return applyDeadzone(config.filter.applyFilter(rawAxis), config.deadzone)
    }

    /**
     * Calculates a normalized (0–360) angle from the y-axis to the joystick's filtered position.
     * This is analogous to a polar theta-value.
     * @return the normalized angle from the y-axis to the joystick location, in degrees.
     */
    val angle: Double
        get() {
            // Compute the angle relative to the y-axis (90°)
            val rawAngle = 90 - Math.toDegrees(Math.atan2(rooGetY(), rooGetX()))
            // Normalize the angle so that it is positive
            return (rawAngle % 360 + 360) % 360
        }

    /**
     * Returns the absolute (positive) distance from the origin by which the joystick has been
     * displaced. This is analogous to a polar r-value.
     * @return the distance the joystick has been moved from the origin.
     */
    val distance: Double
        get() = Math.sqrt(Math.pow(rooGetX(), 2.0) + Math.pow(rooGetY(), 2.0))

    // Axis convenience methods
    fun rooGetX(): Double {
        return getAxis(this.xChannel)
    }

    fun rooGetY(): Double {
        return getAxis(this.yChannel)
    }

    fun rooGetZ(): Double {
        return getAxis(this.zChannel)
    }

    fun rooGetTwist(): Double {
        return getAxis(this.twistChannel)
    }

    fun rooGetThrottle(): Double {
        return getAxis(this.throttleChannel)
    }
}