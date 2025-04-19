package org.firstinspires.ftc.teamcode.robot

import android.graphics.Color
import com.qualcomm.hardware.rev.RevBlinkinLedDriver
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.NormalizedColorSensor
import com.qualcomm.robotcore.hardware.NormalizedRGBA
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.extension.initializeForRunToPosition
import org.firstinspires.ftc.teamcode.extension.runToPosition

private const val HARDWARE_MAP_BLINKEN_LED_DRIVER = "blinkenLedDriver"
private const val HARDWARE_MAP_BUCKET_SERVO_MOTOR = "bucketServo"
private const val HARDWARE_MAP_COLOR_SENSOR = "colorSensor"
private const val HARDWARE_MAP_INTAKE_SLIDE_SERVO_MOTOR = "intakeSlideServo"
private const val HARDWARE_MAP_INTAKE_ARM_MOTOR = "intakeArmMotor"
private const val HARDWARE_MAP_INTAKE_LEFT_SERVO_MOTOR = "intakeLeftServo"
private const val HARDWARE_MAP_INTAKE_RIGHT_SERVO_MOTOR = "intakeRightServo"
private const val HARDWARE_MAP_SLIDE_MOTOR = "slideMotor"

private const val SLIDE_LIFT_VELOCITY = 2100.0
private const val INTAKE_ARM_VELOCITY = 800.0

class Robot(private val opmode: LinearOpMode) : BaseMecanumRobot(opmode) {

    private lateinit var bucketServo: Servo
    private lateinit var intakeArmMotor: DcMotorEx
    private lateinit var intakeLeftServo: CRServo
    private lateinit var intakeRightServo: CRServo
    private lateinit var intakeSlideServo: Servo
    private lateinit var outtakeSlideMotor: DcMotorEx

    private lateinit var colorSensor: NormalizedColorSensor
    private lateinit var blinkinLedDriver: RevBlinkinLedDriver

    private val telemetry by lazy { opmode.telemetry }

    private var armState: ArmState = ArmState.Transfer
    private var bucketState: BucketState = BucketState.INIT
    private var intakeSlideState: IntakeSlideState = IntakeSlideState.IN
    private var outtakeSlideState: OuttakeSlideState = OuttakeSlideState.Collapsed

    /**
     * Robot Initialization
     * Use the hardware map to connect to devices.
     * Perform any set-up all the hardware devices.
     */
    override fun initialize() {
        super.initialize()
        // MAKE SURE YOUR ID'S MATCH YOUR CONFIGURATION
        blinkinLedDriver =
            opmode.hardwareMap.get(RevBlinkinLedDriver::class.java, HARDWARE_MAP_BLINKEN_LED_DRIVER)
        colorSensor =
            opmode.hardwareMap.colorSensor.get(HARDWARE_MAP_COLOR_SENSOR) as NormalizedColorSensor
        bucketServo = opmode.hardwareMap.servo.get(HARDWARE_MAP_BUCKET_SERVO_MOTOR)
        intakeArmMotor = opmode.hardwareMap.dcMotor.get(HARDWARE_MAP_INTAKE_ARM_MOTOR) as DcMotorEx
        intakeLeftServo = opmode.hardwareMap.crservo.get(HARDWARE_MAP_INTAKE_LEFT_SERVO_MOTOR)
        intakeRightServo = opmode.hardwareMap.crservo.get(HARDWARE_MAP_INTAKE_RIGHT_SERVO_MOTOR)
        intakeSlideServo = opmode.hardwareMap.servo.get(HARDWARE_MAP_INTAKE_SLIDE_SERVO_MOTOR)
        outtakeSlideMotor = opmode.hardwareMap.dcMotor.get(HARDWARE_MAP_SLIDE_MOTOR) as DcMotorEx

        outtakeSlideMotor.initializeForRunToPosition(
            outtakeSlideState.position,
            DcMotorSimple.Direction.REVERSE,
            true
        )

        // Initialize the arm motor to zero
        intakeArmMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        intakeArmMotor.direction = DcMotorSimple.Direction.FORWARD
        intakeArmMotor.targetPosition = 0
        intakeArmMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        intakeArmMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
        intakeArmMotor.runToPosition(armState.position, INTAKE_ARM_VELOCITY)

        intakeSlideServo.direction = Servo.Direction.REVERSE
        intakeSlideServo.position = intakeSlideState.position

        bucketServo.direction = Servo.Direction.REVERSE
        bucketServo.position = bucketState.position
    }

    fun isArmIntakeDown(): Boolean =
        armState == ArmState.IntakePickup && intakeArmMotor.currentPosition > 900

    fun moveArm(state: ArmState) {
        armState = state
        intakeArmMotor.runToPosition(state.position, INTAKE_ARM_VELOCITY)
    }

    fun moveBucket() {
        val bucketState = when (bucketState) {
            BucketState.UP -> BucketState.DOWN
            else -> BucketState.UP
        }
        moveBucket(bucketState)
    }

    fun moveBucket(state: BucketState) {
        bucketState = state
        bucketServo.position = state.position
    }

    fun moveIntakeSlideIn() {
        moveIntakeSlide(IntakeSlideState.IN)
    }

    fun moveIntakeSlideOut() {
        moveIntakeSlide(IntakeSlideState.OUT)
    }

    private fun moveIntakeSlide(state: IntakeSlideState) {
        intakeSlideState = state
        intakeSlideServo.position = state.position
    }

    fun moveOuttakeSlide(state: OuttakeSlideState) {
        outtakeSlideState = state
        outtakeSlideMotor.runToPosition(state.position, SLIDE_LIFT_VELOCITY)
    }

    /**
     * Reset the slide motor to the collapsed position if it is not busy and target position is less
     * than or equal to zero. This prevents the slide motor from overheating if it is not used.
     */
    fun resetOuttakeSlideIfNotBusy() {
        // TODO: ALSO CHECK IF THE MOTOR IS MOVING??? OR CURRENT POSITION IS ZERO???
        if (!outtakeSlideMotor.isBusy && (outtakeSlideMotor.targetPosition <= OuttakeSlideState.Collapsed.position || outtakeSlideState == OuttakeSlideState.Collapsed)) {
            telemetry.addData("Resetting outtake motor", outtakeSlideMotor.currentPosition)
            outtakeSlideMotor.initializeForRunToPosition(
                OuttakeSlideState.Collapsed.position,
                DcMotorSimple.Direction.REVERSE,
                true
            )
        }
    }

    fun logTelemetryData() {
        telemetry.addData("Bucket Servo Position", bucketServo.position)
        telemetry.addData("Intake Slide Servo Position", intakeSlideServo.position)
        telemetry.addData("Intake arm motor target position", intakeArmMotor.targetPosition)
        telemetry.addData("Intake arm motor current position", intakeArmMotor.currentPosition)
        telemetry.addData("Outtake Slide not busy", outtakeSlideMotor.isBusy)
        telemetry.addData("Outtake Slide power", outtakeSlideMotor.power)
        telemetry.addData("Outtake Slide motor target position", outtakeSlideMotor.targetPosition)
        telemetry.addData("Outtake Slide motor current position", outtakeSlideMotor.currentPosition)
    }

    fun spinIntakeIn() {
        intakeLeftServo.direction = DcMotorSimple.Direction.FORWARD
        intakeRightServo.direction = DcMotorSimple.Direction.REVERSE
        intakeLeftServo.power = 1.0
        intakeRightServo.power = 1.0
    }

    fun spinIntakeOut() {
        intakeLeftServo.direction = DcMotorSimple.Direction.REVERSE
        intakeRightServo.direction = DcMotorSimple.Direction.FORWARD
        intakeLeftServo.power = 1.0
        intakeRightServo.power = 1.0
    }

    fun stopSpinningIntake() {
        intakeLeftServo.power = 0.0
        intakeRightServo.power = 0.0
    }

    /**
     * Update the Blinkin LED color strip based in the latest color sensor data. This will use
     * the hue to determine which color to display.
     */
    fun updateLedColors() {
        // Get the normalized colors from the sensor
        val colors: NormalizedRGBA = colorSensor.normalizedColors
        // The first element (0) will contain the hue, the second element (1) will contain the
        // saturation, and the third element (2) will contain the value.
        // See http://web.archive.org/web/20190311170843/https://infohost.nmt.edu/tcc/help/pubs/colortheory/web/hsv.html
        // for an explanation of HSV color.
        val hsvValues = FloatArray(3)
        // Update the hsvValues array by passing it to Color.colorToHSV()
        Color.colorToHSV(colors.toColor(), hsvValues)
        val hue = hsvValues[0].toInt()
        val pattern = when (hue) {
            in 20..30 -> RevBlinkinLedDriver.BlinkinPattern.RED
            in 65..85 -> RevBlinkinLedDriver.BlinkinPattern.YELLOW
            in 200..240 -> RevBlinkinLedDriver.BlinkinPattern.BLUE
            else -> RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_OCEAN_PALETTE
        }
        blinkinLedDriver.setPattern(pattern)

        telemetry.addLine()
            .addData("Hue", "%.3f", hsvValues[0])
            .addData("Saturation", "%.3f", hsvValues[1])
            .addData("Value", "%.3f", hsvValues[2])
            .addData("Alpha", "%.3f", colors.alpha)
    }
}