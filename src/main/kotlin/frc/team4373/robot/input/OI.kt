package frc.team4373.robot.input

import edu.wpi.first.wpilibj.buttons.Button
import edu.wpi.first.wpilibj.buttons.JoystickButton
import frc.team4373.robot.RobotMap
import frc.team4373.robot.commands.camera.VisionQuerierCommand
import frc.team4373.robot.commands.drivetrain.DrivetrainCommand
import frc.team4373.robot.commands.drivetrain.ResetNorthCommand
import frc.team4373.robot.commands.drivetrain.RotateAngleOffsetAuton
import frc.team4373.robot.commands.drivetrain.SetDriveModeCommand
import frc.team4373.robot.commands.shooter.ShooterFallbackShootCommand
import frc.team4373.robot.commands.shooter.ShooterShootCommand
import frc.team4373.robot.commands.util.ClearSubsystemsCommandGroup
import frc.team4373.robot.input.filters.*
import frc.team4373.swerve.SwerveDrivetrain

/**
 * OI provides access to operator interface devices.
 */
object OI {
    @JvmStatic fun getInstance(): OI = OI

    /**
     * The drive joystick controlling the robot.
     */
    val driveJoystick: RooJoystick

    /**
     * The operator joystick controlling the robot.
     */
    val operatorJoystick: RooJoystick
    private val resetNorthButton: Button
    private val setNorthUpButton: Button
    private val setOwnShipUpButton: Button
    private val setDriveTrainButton: Button
    private val clearCommandsButton: Button
    private val alignToTargetButton: Button
    private val shootButton: Button
    private val fallbackShootButton: Button
    private val shootFromWallButton: Button
    private val toggleSpinnerButton: Button? = null
    private val spinnerRevsButton: Button? = null
    private val spinnerColorButton: Button? = null

    init {
        //FIXME: These filters need to be tested.
        /*
        FineGrainedPiecewiseFilter: https://www.desmos.com/calculator/3rhniwotk2
        XboxAxisFilter: https://www.desmos.com/calculator/r6t3rzmh2x

        Template for new filters: https://www.desmos.com/calculator/jbb9fc5zwh
         */
        driveJoystick = RooJoystick(RobotMap.DRIVE_JOYSTICK_PORT, LogitechFilter(), RobotMap.JOYSTICK_DEFAULT_DEADZONE)
        driveJoystick.configureAxis(driveJoystick.zChannel, SwerveTwistFilter(), 0.05)
        driveJoystick.configureAxis(driveJoystick.throttleChannel, LogitechSliderAxisFilter(), 0.01)
        operatorJoystick = RooJoystick(RobotMap.OPERATOR_JOYSTICK_PORT, XboxFilter(), RobotMap.JOYSTICK_DEFAULT_DEADZONE)
        operatorJoystick.configureAxis(RobotMap.OPER_ADJUST_SHOOT_SPEED_AXIS, XboxThrottleFilter(), RobotMap.JOYSTICK_DEFAULT_DEADZONE)

        resetNorthButton = JoystickButton(
            driveJoystick,
            RobotMap.DRIVE_RESET_NORTH_BUTTON
        )
        resetNorthButton.whenPressed(ResetNorthCommand())

        setNorthUpButton = JoystickButton(
            driveJoystick,
            RobotMap.DRIVE_NORTH_UP_BUTTON
        )
        setNorthUpButton.whenPressed(
            SetDriveModeCommand(
                SwerveDrivetrain.DriveMode.NORTH_UP
            )
        )

        setOwnShipUpButton = JoystickButton(
            driveJoystick,
            RobotMap.DRIVE_OWN_SHIP_UP_BUTTON
        )
        setOwnShipUpButton.whenPressed(
            SetDriveModeCommand(
                SwerveDrivetrain.DriveMode.OWN_SHIP_UP
            )
        )

        setDriveTrainButton = JoystickButton(
            driveJoystick,
            RobotMap.DRIVE_DISABLE_ASSIST_BUTTON
        )
        setDriveTrainButton.whenPressed(DrivetrainCommand())

        clearCommandsButton = JoystickButton(
            driveJoystick,
            RobotMap.DRIVE_CLEAR_COMMANDS_BUTTON
        )
        clearCommandsButton.whenPressed(ClearSubsystemsCommandGroup())

        alignToTargetButton = JoystickButton(
            driveJoystick,
            RobotMap.DRIVE_VISION_ALIGN_BUTTON
        )
        alignToTargetButton.whenPressed(
            VisionQuerierCommand(RobotMap.VISION_ANG_OFFSET_FIELD, RobotMap.VISION_ALIGN_ALLOWABLE_OFFSET_DEG, { ang -> RotateAngleOffsetAuton(ang) })
        )

        shootButton = JoystickButton(
            operatorJoystick,
            RobotMap.OPER_SHOOT_BUTTON
        )
        shootButton.whileHeld(ShooterShootCommand())

        fallbackShootButton = JoystickButton(
            operatorJoystick,
            RobotMap.OPER_FALLBACK_SHOOT_BUTTON
        )
        fallbackShootButton.whileHeld(ShooterFallbackShootCommand())

        shootFromWallButton = JoystickButton(
            operatorJoystick,
            RobotMap.OPER_SHOOT_FROM_WALL_BUTTON
        )
        shootFromWallButton.whileHeld(
            ShooterShootCommand(
                RobotMap.SHOOT_FROM_WALL_SPEED
            )
        )
        // this.toggleSpinnerButton = new JoystickButton(this.operatorJoystick,
        //         RobotMap.OPER_TOGGLE_SPINNER_BUTTON);
        // this.toggleSpinnerButton.whenPressed(new ToggleSpinnerCommand());
        //
        // this.spinnerRevsButton = new JoystickButton(this.operatorJoystick,
        //         RobotMap.OPER_SPINNER_REVS_BUTTON);
        // this.spinnerRevsButton.whenPressed(new WheelSpinnerRevolutionsCommand(
        //         RobotMap.SPINNER_TARGET_REVS));
        //
        // this.spinnerColorButton = new JoystickButton(this.operatorJoystick,
        //         RobotMap.OPER_SPINNER_COLOR_BUTTON);
        // this.spinnerColorButton.whenPressed(new WheelSpinnerColorCommand());
    }
}