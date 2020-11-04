package frc.team4373.robot.subsystems

import com.ctre.phoenix.sensors.PigeonIMU
import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.geometry.Translation2d
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team4373.robot.Robot
import frc.team4373.robot.RobotMap
import frc.team4373.robot.fromPigeonUnits


/**
 * A Javadoc template. TODO: Update Drivetrain Javadoc.
 */
object Drivetrain: SubsystemBase() {
    @JvmStatic fun getInstance(): Drivetrain = Drivetrain

    /**
     * The drive mode for the swerve bot.
     *
     *
     * The modes are as follows:
     *
     *  * North-up mode maintains a constant sense of north using the gyro (i.e., pushing the
     * joystick forward moves in the same direction regardless of the robot's orientation)
     *  * Own-ship-up mode always drives relative to the front of the robot
     * (i.e., pushing the joystick forward moves the robot in the direction it is
     * facing)
     *
     */
    enum class DriveMode {
        NORTH_UP, OWN_SHIP_UP
    }

    /**
     * The swerve brake mode for the swerve bot (i.e., what to do when the input is zero).
     *
     *
     * The modes are as follows:
     *
     *  * Implode mode points all wheels inward (toward the center of the bot)
     *  * Octagon mode points all wheels orthogonally to the diagonal that runs from the wheel
     * to the center (the standard turning configuration)
     *  * When swerve brake mode is disabled, the wheels remain in the direction they were
     * facing before stopping
     *
     */
    enum class BrakeMode {
        IMPLODE, OCTAGON, NONE
    }

    val kinematics: SwerveDriveKinematics
    private val odometry: SwerveDriveOdometry
    private val pigeon: PigeonIMU
    private val initialAngle: Double
    private val frontLeft: SwerveWheel
    private val frontRight: SwerveWheel
    private val backRight: SwerveWheel
    private val backLeft: SwerveWheel
    var driveMode: DriveMode = DriveMode.NORTH_UP
    var brakeMode: BrakeMode = BrakeMode.IMPLODE
        set(value) {
            field = value
            brakeStates = when(value) {
                BrakeMode.IMPLODE -> kinematics.toSwerveModuleStates(ChassisSpeeds(0.0, 0.0, 1.0)).map { it.speedMetersPerSecond = 0.0; return@map it }.reversed().toTypedArray()
                BrakeMode.OCTAGON -> kinematics.toSwerveModuleStates(ChassisSpeeds(0.0, 0.0, 1.0)).map { it.speedMetersPerSecond = 0.0; return@map it }.toTypedArray()
                BrakeMode.NONE -> null
            }
        }
    private var brakeStates: Array<SwerveModuleState>? = null
    init {
        val frontLeft = Translation2d(15.0, 15.0)
        val frontRight = Translation2d(15.0, -15.0)
        val backRight = Translation2d(-15.0, -15.0)
        val backLeft = Translation2d(-15.0, 15.0)
        kinematics = SwerveDriveKinematics(frontLeft, frontRight, backRight, backLeft)
        odometry = SwerveDriveOdometry(kinematics, Rotation2d(), Pose2d())

        pigeon = PigeonIMU(RobotMap.PIGEON_ID)
        initialAngle = getPigeonYawRaw()

        this.frontLeft = SwerveWheel(RobotMap.left1Drive, RobotMap.left1Rotate)
        this.frontRight = SwerveWheel(RobotMap.right1Drive, RobotMap.right1Rotate)
        this.backRight = SwerveWheel(RobotMap.right2Drive, RobotMap.right2Rotate)
        this.backLeft = SwerveWheel(RobotMap.left2Drive, RobotMap.left2Rotate)
    }

    override fun periodic() {
        odometry.update(Rotation2d.fromDegrees(getPigeonYawRaw()), frontLeft.state, frontRight.state, backRight.state, backLeft.state)
    }

    /**
    * Returns the raw Pigeon yaw value.
    * @return Pigeon yaw value.
    */
    public fun getPigeonYawRaw(): Double {
        val ypr = DoubleArray(3)
        pigeon.getYawPitchRoll(ypr)
        return ypr[0] * -1
    }

//    /**
//     * Resets the pigeon's yaw to consider the current orientation field-forward (zero degrees).
//     */
//    fun resetPigeonYaw() {
//        initialAngle = getPigeonYawRaw()
//    }
//
//    /**
//     * Sets the pigeon's yaw to be a given value at the current position.
//     * @param angle the value, in degrees, that the pigeon should have at the current position..
//     */
//    fun setPigeonYaw(angle: Double) {
//        initialAngle = getPigeonYawRaw() + angle
//    }

    fun drive(rotation: Double, x: Double, y: Double) {
        val swerveModuleStates = when(driveMode) {
            DriveMode.NORTH_UP -> kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(x, y, rotation, Rotation2d.fromDegrees(getPigeonYawRaw())/*fromPigeonUnits(getPigeonYawRaw())*/))
            DriveMode.OWN_SHIP_UP -> kinematics.toSwerveModuleStates(ChassisSpeeds(x, y, rotation))
        }
        if (swerveModuleStates != null) {
            frontLeft.state = swerveModuleStates[0]
            frontRight.state = swerveModuleStates[1]
            backRight.state = swerveModuleStates[2]
            backLeft.state = swerveModuleStates[3]
        }
    }

    fun setModuleStates(desiredStates: Array<SwerveModuleState>) {
        SwerveDriveKinematics.normalizeWheelSpeeds(desiredStates,
                RobotMap.MAX_SPEED_METERS_PER_SECOND)
        frontLeft.state = desiredStates[0]
        frontRight.state = desiredStates[1]
        backLeft.state = desiredStates[2]
        backRight.state = desiredStates[3]
    }

    val pose: Pose2d
        get() = odometry.poseMeters

    fun stop() {
        frontLeft.stop()
        frontRight.stop()
        backLeft.stop()
        backRight.stop()
    }

    fun brake() {
        brakeStates?.let { setModuleStates(it) }
    }
}
/*

/**
 * A programmatic representation of a swerve drivetrain.
 *
 *
 * Requirements:
 *
 *  * The robot must be rectangular.
 *  * The robot must be rectangular.
 *  * The motors must use CTRE TalonSRX motor controllers.
 *  * The robot must have a Pigeon IMU.
 *  * The motor controllers and Pigeon must be connected over the CAN bus.
 *  * The motor controlllers must have encoders connected.
 *
 */
abstract class SwerveDrivetrain protected constructor(config: SwerveConfig) : Subsystem() {
    /**
     * A programmatic representation of which wheel is being referenced.
     * Typically, "1" wheels are front and "2" wheels are rear.
     */
    enum class WheelID {
        RIGHT_1, RIGHT_2, LEFT_1, LEFT_2
    }

    /**
     * The drive mode for the swerve bot.
     *
     *
     * The modes are as follows:
     *
     *  * North-up mode maintains a constant sense of north using the gyro (i.e., pushing the
     * joystick forward moves in the same direction regardless of the robot's orientation)
     *  * Own-ship-up mode always drives relative to the front of the robot
     * (i.e., pushing the joystick forward moves the robot in the direction it is
     * facing)
     *
     */
    enum class DriveMode {
        NORTH_UP, OWN_SHIP_UP
    }

    /**
     * The swerve brake mode for the swerve bot (i.e., what to do when the input is zero).
     *
     *
     * The modes are as follows:
     *
     *  * Implode mode points all wheels inward (toward the center of the bot)
     *  * Octagon mode points all wheels orthogonally to the diagonal that runs from the wheel
     * to the center (the standard turning configuration)
     *  * When swerve brake mode is disabled, the wheels remain in the direction they were
     * facing before stopping
     *
     */
    enum class BrakeMode {
        IMPLODE, OCTAGON, NONE
    }

    /**
     * The [SwerveInputTransform] transform object for this swerve bot.
     */
    var transform: SwerveInputTransform
    private val right1: SwerveWheel
    private val right2: SwerveWheel
    private val left1: SwerveWheel
    private val left2: SwerveWheel
    private val pigeon: PigeonIMU
    private var initialAngle: Double
    /**
     * Gets the current drive mode.
     * @return the currently selected [DriveMode].
     */
    /**
     * Sets the swerve drive mode (north- or own-ship-up).
     * @param driveMode the [DriveMode] to set.
     */
    var driveMode = DriveMode.NORTH_UP
    private var brakeMode = BrakeMode.IMPLODE
    private var brakeVectors: WheelVector.VectorSet

    /**
     * Stops the robot (i.e., sets outputs of all motors of all wheels to zero).
     */
    fun stop() {
        right1.stop()
        right2.stop()
        left1.stop()
        left2.stop()
    }

    /**
     * Drives using the transform for the given parameters.
     * @param rotation the joystick rotation, in degrees.
     * @param x the x-coordinate, -1 to 1.
     * @param y the y-coordinate, -1 to 1.
     */
    fun drive(rotation: Double, x: Double, y: Double) {
        when (driveMode) {
            DriveMode.NORTH_UP -> setWheelsPID(transform.processNorthUp(rotation, x, y, angle))
            DriveMode.OWN_SHIP_UP -> setWheelsPID(transform.processOwnShipUp(rotation, x, y))
            else -> {
            }
        }
    }

    /**
     * Sets the wheels to the configured brake configuration if any is selected.
     */
    fun brake() {
        setWheelsPID(brakeVectors)
    }

    /**
     * Sets velocity vectors to the four SwerveWheels with PID setpoints for both speed and angle.
     * @param vectors the four vectors ordered right1, left1, left2, right2.
     */
    fun setWheelsPID(vectors: WheelVector.VectorSet?) {
        if (vectors == null) return
        if (vectors.right1 != null) right1.set(vectors.right1.speed, vectors.right1.angle)
        if (vectors.right2 != null) right2.set(vectors.right2.speed, vectors.right2.angle)
        if (vectors.left1 != null) left1.set(vectors.left1.speed, vectors.left1.angle)
        if (vectors.left2 != null) left2.set(vectors.left2.speed, vectors.left2.angle)
    }

    /**
     * Sets vectors to the SwerveWheels with a PID setpoint for angle and % output for speed.
     * @param vectors the four vectors ordered right1, left1, left2, right2.
     */
    fun setWheelsPercOut(vectors: WheelVector.VectorSet?) {
        if (vectors == null) return
        if (vectors.right1 != null) {
            right1.setPercentOutput(vectors.right1.speed, vectors.right1.angle)
        }
        if (vectors.right2 != null) {
            right2.setPercentOutput(vectors.right2.speed, vectors.right2.angle)
        }
        if (vectors.left1 != null) {
            left1.setPercentOutput(vectors.left1.speed, vectors.left1.angle)
        }
        if (vectors.left2 != null) {
            left2.setPercentOutput(vectors.left2.speed, vectors.left2.angle)
        }
    }

    /**
     * Returns the current angle relative to the starting position (mod 360).
     * @return the current angle relative to the starting position on the interval [0, 360).
     */
    val angle: Double
        get() = Utils.normalizeAngle(pigeonYawRaw - initialAngle)

    /**
     * Returns the raw Pigeon yaw value.
     * @return Pigeon yaw value.
     */
    val pigeonYawRaw: Double
        get() {
            val ypr = DoubleArray(3)
            pigeon.getYawPitchRoll(ypr)
            return ypr[0] * -1
        }

    /**
     * Gets the percent output of the specified SwerveWheel.
     * @param wheelID the ID of the wheel.
     * @return the wheel's percent output, [-1, 1].
     */
    fun getPercentOutput(wheelID: WheelID): Double {
        return getWheel(wheelID).drivePercentOutput()
    }

    /**
     * Gets the current position of the drive motor.
     * @param wheelID the ID of the wheel whose position to fetch.
     * @return the current position in encoder units.
     */
    fun getDriveMotorPosition(wheelID: WheelID): Double {
        return getWheel(wheelID).getDriveMotorPosition()
    }

    /**
     * Gets the average of the drive motor positions.
     * @return average of drive position in encoder units.
     */
    val averageDriveMotorPosition: Double
        get() = (getDriveMotorPosition(WheelID.LEFT_1)
                + getDriveMotorPosition(WheelID.LEFT_2)
                + getDriveMotorPosition(WheelID.RIGHT_1)
                + getDriveMotorPosition(WheelID.RIGHT_2)) / SwerveConstants.WHEEL_COUNT

    /**
     * Gets the velocity of the drive motor for the specified wheel.
     * @param wheelID the ID of the wheel whose drive velocity to fetch.
     * @return the wheel's drive velocity in encoder units.
     */
    fun getDriveMotorVelocity(wheelID: WheelID): Double {
        return getWheel(wheelID).getDriveMotorVelocity()
    }

    /**
     * Gets the position of the rotator motor for the specified wheel.
     * Since the rotator is a mag encoder, this is an absolute position.
     *
     * @param wheelID the ID of the wheel whose rotator position to fetch.
     * @return the wheel's absolute rotator position in encoder units.
     */
    fun getRotatorMotorPosition(wheelID: WheelID): Double {
        return getWheel(wheelID).getRotatorMotorPosition()
    }

    /**
     * Gets the velocity of the rotator motor for the specified wheel.
     * @param wheelID the ID of the wheel whose rotator velocity to fetch.
     * @return the wheel's rotator velocity in encoder units.
     */
    fun getRotatorMotorVelocity(wheelID: WheelID): Double {
        return getWheel(wheelID).getRotatorMotorVelocity()
    }

    /**
     * Sets the PID gains for a specified wheel.
     * @param wheelID the wheel whose PID gains to change.
     * @param drivePID a [SwerveConfig.PID] object with new parameters for the drive PID,
     * or null to leave unchanged.
     * @param rotatorPID a [SwerveConfig.PID] object with parameters for rotational PID,
     * or null to leave unchanged.
     */
    fun setPID(wheelID: WheelID, drivePID: SwerveConfig.PID?, rotatorPID: SwerveConfig.PID?) {
        getWheel(wheelID).setPID(drivePID, rotatorPID)
    }

    /**
     * Sets the swerve brake mode.
     * @param brakeMode the [BrakeMode] to set.
     */
    fun setBrakeMode(brakeMode: BrakeMode) {
        this.brakeMode = brakeMode
        brakeVectors = transform.calculateBrakeVectors(brakeMode)
    }

    /**
     * Gets the currently selected swerve brake mode.
     * @return the currently selected [BrakeMode].
     */
    fun getBrakeMode(): BrakeMode {
        return brakeMode
    }

    /**
     * Gets the swerve wheel with the specified ID.
     * @param wheelID the ID of the wheel to fetch.
     * @return the [SwerveWheel] object.
     */
    private fun getWheel(wheelID: WheelID): SwerveWheel {
        return when (wheelID) {
            WheelID.RIGHT_1 -> right1
            WheelID.RIGHT_2 -> right2
            WheelID.LEFT_1 -> left1
            WheelID.LEFT_2 -> left2
            else -> getWheel(WheelID.RIGHT_1)
        }
    }

    /**
     * Resets the encoders to within [-4095, 4095]
     * Call this method in `robotInit` to (almost) eliminate the possibility of
     * accumulator rollover during one power cycle.
     */
    fun modularizeEncoders() {
        right1.modularizeAbsoluteEncoder()
        right2.modularizeAbsoluteEncoder()
        left1.modularizeAbsoluteEncoder()
        left2.modularizeAbsoluteEncoder()
    }

    /**
     * Resets the position of the rotator encoder of the given wheel.
     *
     *
     * This function should **NEVER** *regularly* be called.
     * It should be called once per mechanical change, with all wheels facing forward.
     * @param wheelID the wheel whose encoder should be reset.
     */
    fun resetEncoder(wheelID: WheelID) {
        getWheel(wheelID).resetAbsoluteEncoder()
    }

    /**
     * Resets the pigeon's yaw to consider the current orientation field-forward (zero degrees).
     */
    fun resetPigeonYaw() {
        initialAngle = pigeonYawRaw
    }

    /**
     * Sets the pigeon's yaw to be a given value at the current position.
     * @param angle the value, in degrees, that the pigeon should have at the current position..
     */
    fun setPigeonYaw(angle: Double) {
        initialAngle = pigeonYawRaw + angle
    }

    /**
     * Creates a new SwerveDrivetrain subclass instance with the given configuration.
     * @param config the configuration for the new SwerveDrivetrain.
     */
    init {
        right1 = SwerveWheel(WheelID.RIGHT_1,
                config.wheels.right1Drive, config.wheels.right1Rotate,
                config.wheels.maxWheelSpeed, config.wheels.currentLimitConfig)
        right2 = SwerveWheel(WheelID.RIGHT_2,
                config.wheels.right2Drive, config.wheels.right2Rotate,
                config.wheels.maxWheelSpeed, config.wheels.currentLimitConfig)
        left1 = SwerveWheel(WheelID.LEFT_1,
                config.wheels.left1Drive, config.wheels.left1Rotate,
                config.wheels.maxWheelSpeed, config.wheels.currentLimitConfig)
        left2 = SwerveWheel(WheelID.LEFT_2,
                config.wheels.left2Drive, config.wheels.left2Rotate,
                config.wheels.maxWheelSpeed, config.wheels.currentLimitConfig)
        pigeon = PigeonIMU(config.pigeonID)
        initialAngle = pigeonYawRaw
        transform = SwerveInputTransform(config.dimensions.trackwidth,
                config.dimensions.wheelbase)
        brakeVectors = transform.calculateBrakeVectors(brakeMode)
    }
}
*/