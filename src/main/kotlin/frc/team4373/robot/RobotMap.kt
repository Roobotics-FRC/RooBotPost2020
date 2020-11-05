//@file:JvmName("RobotMap")
//package frc.team4373.robot
//
//import com.ctre.phoenix.motorcontrol.NeutralMode
//import edu.wpi.first.wpilibj.I2C
//import frc.team4373.swerve.SwerveConfig
//import frc.team4373.swerve.SwerveConfig.*
//
//private val drivePID = SwerveConfig.PID(0.0, 0.5, 0.0, 0.2)
//private val rotatePID = SwerveConfig.PID(0.0, 1.0, 0.0, 0.05)
//
///**
// * The config for the swerve drivetrain.
// */
//val swerveConfig: SwerveConfig = SwerveConfig(
//    RobotDimensions(30.0, 30.0),
//    WheelsConfig(
//        MotorConfig(17, true, NeutralMode.Brake, true, drivePID),
//        MotorConfig(17, true, NeutralMode.Brake, false, rotatePID),
//        MotorConfig(17, false, NeutralMode.Brake, true, drivePID),
//        MotorConfig(17, true, NeutralMode.Brake, false, rotatePID),
//        MotorConfig(17, true, NeutralMode.Brake, true, drivePID),
//        MotorConfig(17, true, NeutralMode.Brake, false, rotatePID),
//        MotorConfig(17, false, NeutralMode.Brake, true, drivePID),
//        MotorConfig(17, true, NeutralMode.Brake, false, rotatePID),
//        7000.0,
//        CurrentLimitConfig.NONE
//    ),
//    19
//)
//
//// OI devices
//const val DRIVE_JOYSTICK_PORT: Int = 0
//const val OPERATOR_JOYSTICK_PORT: Int = 1
//const val JOYSTICK_DEFAULT_DEADZONE: Double = 0.09
//
//// Buttons and axes
//const val DRIVE_RESET_NORTH_BUTTON: Int = 7
//const val DRIVE_NORTH_UP_BUTTON: Int = 10
//const val DRIVE_OWN_SHIP_UP_BUTTON: Int = 12
//const val DRIVE_SLOWER_SPEED_BUTTON: Int = 2
//const val DRIVE_DISABLE_ASSIST_BUTTON: Int = 5
//const val DRIVE_CLEAR_COMMANDS_BUTTON: Int = 11
//const val DRIVE_VISION_ALIGN_BUTTON: Int = 6
//const val DRIVE_DISABLE_BRAKE_BUTTON: Int = 1 // drive trigger
//const val OPER_BALL_RELEASE_BUTTON: Int = 5 // left bumper
//const val OPER_INTAKE_BUTTON: Int = 6 // right bumper
//const val OPER_RAISE_L_WINCH_AXIS: Int = 2 // L trigger
//const val OPER_RAISE_R_WINCH_AXIS: Int = 3 // R trigger
//const val OPER_SHOOT_BUTTON: Int = 4 // Y button
//const val OPER_FALLBACK_SHOOT_BUTTON: Int = 2 // B button
//const val OPER_TOGGLE_SPINNER_BUTTON: Int = 10 // right stick click
//const val OPER_SPINNER_REVS_BUTTON: Int = 7 // back button
//const val OPER_SPINNER_COLOR_BUTTON: Int = 8 // start button
//const val OPER_REVERSE_INTAKE_BUTTON: Int = 1 // A button
//const val OPER_ADJUST_SHOOT_SPEED_AXIS: Int = 1 // left stick Y
//const val OPER_SHOOT_FROM_WALL_BUTTON: Int = 3 // X button
//
//const val OPER_ROTATE_VIB_INTENSITY: Double = 0.5
//
//// Speed constants
//const val CLIMB_ELEVATOR_MOVE_SPEED: Double = 1.0
//const val CLIMB_WINCH_MAX_SPEED: Double = 0.8
//const val SHOOTER_MAX_SPEED_NATIVE_UNITS: Double = 100000.0
//const val DRIVE_SLOWER_SPEED_FACTOR: Double = 4.0
//const val AUTON_TURN_SPEED: Double = 0.25
//const val AUTON_LINE_SHOOT_SPEED: Double = 0.715
//const val AUTON_DRIVE_SPEED: Double = 0.2
//const val GROUND_INTAKE_SPEED: Double = 1.0
//const val UPTAKE_INTAKE_SPEED: Double = 0.45
//const val DRIVE_ASSIST_MAX_TURN_SPEED: Double = 0.2
//const val SHOOT_FROM_WALL_SPEED: Double = 0.63
//
//// Non-motor devices
//const val INTAKE_RELEASE_SERVO_PORT: Int = 9
//const val BOTTOM_LIMIT_SWITCH_DIO_PORT: Int = 0
//const val TOP_LIMIT_SWITCH_DIO_PORT: Int = 1
//const val WHEEL_SPINNER_DEPLOY_SERVO_PORT: Int = 8
//val COLOR_SENSOR_PORT: I2C.Port = I2C.Port.kOnboard
//
//// Physical state constants
//const val INTAKE_SERVO_RELEASE_ANGLE: Double = 1.0
//const val INTAKE_SERVO_RETAIN_ANGLE: Double = 0.0
//const val SPINNER_SERVO_DEPLOY_ANGLE: Double = 0.5
//const val SPINNER_SERVO_RETRACT_ANGLE: Double = 0.0
//const val SPINNER_SPEED: Double = 0.2
//const val SPINNER_TARGET_REVS: Int = 4
//const val DRIVE_WHEEL_DIAMETER_IN: Double = 4.0
//const val DRIVE_GEARBOX_RATIO: Double = 20 / 3.0
//const val SHOOTER_TIME_TO_SPIN_UP_SEC: Double = 1.0
//// the below offset necessitates a CLOCKWISE (positive) rotation
//const val INTER_CAMERA_SHOOTER_DIST_IN: Double = 4.25
//
//// Talon constants
//const val PID_IDX: Int = 0
//const val DRIVE_ENCODER_COUNTS_PER_REV: Double = 4096.0
//
//// Conversion factors
//const val ENCODER_UNITS_TO_INCHES: Double =
//    DRIVE_WHEEL_DIAMETER_IN * Math.PI / DRIVE_ENCODER_COUNTS_PER_REV / DRIVE_GEARBOX_RATIO
//
//// Colors
//const val RED_THRESHOLD: Double = 0.4
//const val BLUE_THRESHOLD: Double = 0.2
//const val GREEN_THRESHOLD: Double = 0.5
//
//// Vision
//const val VISION_SAMPLE_COUNT: Double = 10.0
//const val VISION_TABLE_NAME: String = "Vision"
//const val VISION_ANG_OFFSET_FIELD: String = "degree_offset"
//const val VISION_DIST_FIELD: String = "current_distance"
//const val VISION_ALIGN_ALLOWABLE_OFFSET_DEG: Double = 1.0
//const val MAX_TURN_AUTON_TIME_SEC: Double = 3.0
//const val INTER_QUERY_DELAY_SEC: Double = 0.45
//const val MAX_ALLOWABLE_VISION_ITERATIONS: Int = 4
//
//// Motor configurations
//val SHOOTER_MOTOR_1_CONFIG = MotorConfig(
//    21, true, NeutralMode.Brake, false,
//    PID(0.0, 0.05, 0.0, 0.05)
//)
//val SHOOTER_MOTOR_2_CONFIG = MotorConfig(22, true, NeutralMode.Brake)
//val CLIMB_LIFT_CONFIG = MotorConfig(41, true, NeutralMode.Brake)
//val CLIMB_LEFT_WINCH_CONFIG = MotorConfig(42, false, NeutralMode.Brake)
//val CLIMB_RIGHT_WINCH_CONFIG = MotorConfig(43, false, NeutralMode.Brake)
//val GROUND_INTAKE_MOTOR_CONFIG = MotorConfig(31, true, NeutralMode.Coast)
//val UPTAKE_INTAKE_MOTOR_CONFIG = MotorConfig(32, false, NeutralMode.Coast)
//val WHEEL_SPINNER_MOTOR_CONFIG = MotorConfig(51, false, NeutralMode.Brake)
//
//// Timeouts
//const val SPINNER_DEPLOY_TIME_SEC: Double = 1.5
//const val AUTON_SHOOT_TIME_SEC: Double = 5.0
//
//// time to wait before engaging drive-straight assist (ms)
//const val DRIVE_STRAIGHT_COOLDOWN_MS: Double = 500.0
//
//// Programmatic resources
//const val FP_EQUALITY_THRESHOLD: Double = 1e-5
//
//// PID
//val DRIVE_STRAIGHT_ROTATE_GAINS: PID = PID(0.0, 0.1, 0.0, 0.0)
//
//data class MotorConfig @JvmOverloads constructor(val id: Int, val inverted: Boolean, val neutralMode: NeutralMode, val encoderPhase: Boolean, val gains: PID = PID(0.0, 0.0, 0.0, 0.0))
//// Utility classes
//class MotorConfig {
//    val id: Int
//    val inverted: Boolean
//    val neutralMode: NeutralMode
//    val encoderPhase: Boolean
//    val gains: PID
//
//    /**
//     * Constructs a new MotorConfig for a motor using closed-loop control.
//     * @param id the CAN ID of the motor.
//     * @param inverted whether to invert motor output values.
//     * @param neutralMode the motor's neutral mode.
//     * @param encoderPhase whether the motor is out of phase with its sensor.
//     * @param gains the PID gains for this motor's closed-loop control.
//     */
//    constructor(
//        id: Int, inverted: Boolean,
//        neutralMode: NeutralMode, encoderPhase: Boolean, gains: PID
//    ) {
//        this.id = id
//        this.inverted = inverted
//        this.neutralMode = neutralMode
//        this.encoderPhase = encoderPhase
//        this.gains = gains
//    }
//
//    /**
//     * Constructs a new MotorConfig for a motor not under closed-loop control.
//     * @param id the CAN ID of the motor.
//     * @param inverted whether to invert motor output values.
//     * @param neutralMode the motor's neutral mode.
//     */
//    constructor(id: Int, inverted: Boolean, neutralMode: NeutralMode) {
//        this.id = id
//        this.inverted = inverted
//        this.neutralMode = neutralMode
//        encoderPhase = false
//        gains = PID(0, 0, 0, 0)
//    }
//}
//
//// PID- and sensor-related constants
//class PID
///**
// * Constructs a new PID parameters object.
// * @param kF feed-forward gain.
// * @param kP proportional gain.
// * @param kI integral gain.
// * @param kD derivative gain.
// */(val kF: Double, val kP: Double, val kI: Double, val kD: Double)