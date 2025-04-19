package org.firstinspires.ftc.teamcode.robot

import androidx.annotation.CallSuper
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import kotlin.math.abs
import kotlin.math.max

private const val DEFAULT_DRIVE_SPEED: Double = 0.6

private const val HARDWARE_MAP_FRONT_LEFT_MOTOR = "frontLeftMotor"
private const val HARDWARE_MAP_FRONT_RIGHT_MOTOR = "frontRightMotor"
private const val HARDWARE_MAP_BACK_LEFT_MOTOR = "backLeftMotor"
private const val HARDWARE_MAP_BACK_RIGHT_MOTOR = "backRightMotor"

private const val TELEMETRY_KEY_SPEED = "Speed"
private const val TELEMETRY_KEY_Y_VALUE = "Axial Y Value"

open class BaseMecanumRobot(private val opmode: LinearOpMode) {

    // DECLARE OUR DRIVE MOTORS
    private lateinit var frontLeftMotor: DcMotor
    private lateinit var backLeftMotor: DcMotor
    private lateinit var frontRightMotor: DcMotor
    private lateinit var backRightMotor: DcMotor

    private val telemetry by lazy { opmode.telemetry }

    /**
     * Robot Initialization
     * Use the hardware map to connect to devices.
     * Perform any set-up all the hardware devices.
     */
    @CallSuper
    open fun initialize() {
        // MAKE SURE YOUR ID'S MATCH YOUR CONFIGURATION
        frontLeftMotor = opmode.initializeDriveMotor(HARDWARE_MAP_FRONT_LEFT_MOTOR)
        backLeftMotor = opmode.initializeDriveMotor(HARDWARE_MAP_BACK_LEFT_MOTOR, DcMotorSimple.Direction.REVERSE)
        frontRightMotor = opmode.initializeDriveMotor(HARDWARE_MAP_FRONT_RIGHT_MOTOR)
        backRightMotor = opmode.initializeDriveMotor(HARDWARE_MAP_BACK_RIGHT_MOTOR)
    }

    /**
     * Move the robot
     * @param axial Driving forward and backward - Left-joystick Forward/Backward
     * @param lateral Strafing right and left - Left-joystick Right and Left
     * @param yaw Rotating Clockwise and counter clockwise - Right-joystick Right and Left
     * @param powerMultiplier Multiplier to increase default speed (a.k.a, turbo) - Left-trigger
     * @param powerReducer Decrease default speed - Right-trigger
     */
    fun drive(
        axial: Double,
        lateral: Double,
        yaw: Double,
        powerMultiplier: Double = 0.0,
        powerReducer: Double = 0.0
    ) {
        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        val denominator: Double = max(abs(axial) + abs(lateral) + abs(yaw), 1.0)
        val frontLeftPower = (axial + lateral + yaw) / denominator
        val backLeftPower = (axial - lateral + yaw) / denominator
        val frontRightPower = (axial - lateral - yaw) / denominator
        val backRightPower = (axial + lateral - yaw) / denominator

        // Limit speed to MaxPower
        val maxPower: Double = when {
            powerReducer > 0.0 -> {
                val reducer = if (powerReducer > 0.8) 0.8 else powerReducer
                DEFAULT_DRIVE_SPEED - ((1 - DEFAULT_DRIVE_SPEED) * reducer)
            }

            powerMultiplier > 0.0 -> DEFAULT_DRIVE_SPEED + ((1 - DEFAULT_DRIVE_SPEED) * powerMultiplier)
            else -> DEFAULT_DRIVE_SPEED
        }

        frontLeftMotor.power = frontLeftPower * maxPower
        backLeftMotor.power = backLeftPower * maxPower
        frontRightMotor.power = frontRightPower * maxPower
        backRightMotor.power = backRightPower * maxPower

        telemetry.addData("Axes D:S:Y", "%5.2f %5.2f %5.2f", axial, lateral, yaw)
        telemetry.addData(
            "Wheels lf:rf:lb:rb",
            "%5.2f %5.2f %5.2f %5.2f",
            frontLeftPower,
            frontRightPower,
            backLeftPower,
            backRightPower
        )
        telemetry.addData(TELEMETRY_KEY_SPEED, frontLeftMotor.power)
        telemetry.addData(TELEMETRY_KEY_Y_VALUE, axial)
    }

    private fun OpMode.initializeDriveMotor(
        deviceName: String,
        direction: DcMotorSimple.Direction = DcMotorSimple.Direction.FORWARD
    ): DcMotor = hardwareMap.dcMotor.get(deviceName).apply {
        this.direction = direction
        this.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    }
}