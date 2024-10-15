package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import kotlin.math.abs
import kotlin.math.max

private const val DRIVE_SPEED: Double = 0.5

private const val HARDWARE_MAP_FRONT_LEFT_MOTOR = "frontLeftMotor"
private const val HARDWARE_MAP_FRONT_RIGHT_MOTOR = "frontRightMotor"
private const val HARDWARE_MAP_BACK_LEFT_MOTOR = "backLeftMotor"
private const val HARDWARE_MAP_BACK_RIGHT_MOTOR = "backRightMotor"

private const val TELEMETRY_KEY_ROTATIONS = "Rotations"
private const val TELEMETRY_KEY_SPEED = "Speed"
private const val TELEMETRY_KEY_TRIGGER = "Trigger"
private const val TELEMETRY_KEY_Y_VALUE = "Y Value"

@TeleOp(name = "Team Sapphire: Mecanum TeleOp", group = "Robot")
class MecanumTeleOp : LinearOpMode() {

    // DECLARE OUR MOTORS
    // MAKE SURE YOUR ID'S MATCH YOUR CONFIGURATION
    private val frontLeftMotor: DcMotor by lazy {
        hardwareMap.dcMotor.get(HARDWARE_MAP_FRONT_LEFT_MOTOR)
    }
    private val backLeftMotor: DcMotor by lazy {
        hardwareMap.dcMotor.get(HARDWARE_MAP_BACK_LEFT_MOTOR)
    }
    private val frontRightMotor: DcMotor by lazy {
        hardwareMap.dcMotor.get(HARDWARE_MAP_FRONT_RIGHT_MOTOR)
    }
    private val backRightMotor: DcMotor by lazy {
        hardwareMap.dcMotor.get(HARDWARE_MAP_BACK_RIGHT_MOTOR)
    }

    override fun runOpMode() {
        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        frontRightMotor.direction = DcMotorSimple.Direction.REVERSE
        backRightMotor.direction = DcMotorSimple.Direction.REVERSE

        // Setting zeroPowerBehavior to BRAKE enables a "brake mode".
        // This causes the motor to slow down much faster when it is coasting.
        // This creates a much more controllable drivetrain. As the robot
        // stops much quicker.
        frontLeftMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        frontRightMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        backLeftMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        backRightMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        /* Send telemetry message to signify robot waiting */
        telemetry.addLine("Robot Ready")
        telemetry.update()

        waitForStart()

        if (isStopRequested) return

        while (opModeIsActive()) {
            // START SETUP MECANUM DRIVETRAIN MOTORS
            // Remember, Y stick value is reversed
            val leftStickY: Double = gamepad1.left_stick_y.toDouble()
            val leftStickX: Double = gamepad1.left_stick_x.toDouble()
            val rightStickX: Double = gamepad1.right_stick_x.toDouble()

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            val denominator: Double = max(abs(leftStickY) + abs(leftStickX) + abs(rightStickX), 1.0)
            val frontLeftPower = (leftStickY + leftStickX + rightStickX) / denominator
            val backLeftPower = (leftStickY - leftStickX + rightStickX) / denominator
            val frontRightPower = (leftStickY - leftStickX - rightStickX) / denominator
            val backRightPower = (leftStickY + leftStickX - rightStickX) / denominator

            // Limit speed to MaxPower
            val maxPower: Double = if (gamepad1.left_trigger == 0F) {
                DRIVE_SPEED
            } else {
                DRIVE_SPEED + ((1 - DRIVE_SPEED) * gamepad1.left_trigger)
            }

            frontLeftMotor.power = -frontLeftPower * maxPower
            backLeftMotor.power = backLeftPower * maxPower
            frontRightMotor.power = frontRightPower * maxPower
            backRightMotor.power = backRightPower * maxPower
            // END SETUP MECANUM DRIVETRAIN MOTORS

            // ADD TELEMETRY DATA AND UPDATE
            telemetry.addData(TELEMETRY_KEY_ROTATIONS, frontLeftMotor.currentPosition)
            telemetry.addData(TELEMETRY_KEY_SPEED, frontLeftMotor.power)
            telemetry.addData(TELEMETRY_KEY_TRIGGER, gamepad1.right_trigger)
            telemetry.addData(TELEMETRY_KEY_Y_VALUE, leftStickY)
            telemetry.update()
        }
    }
}