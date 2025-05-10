package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Gamepad
import org.firstinspires.ftc.teamcode.robot.ArmState
import org.firstinspires.ftc.teamcode.robot.BucketState
import org.firstinspires.ftc.teamcode.robot.OuttakeSlideState
import org.firstinspires.ftc.teamcode.robot.Robot

@TeleOp(name = "Team Sapphire: Mecanum TeleOp", group = "Robot")
class MecanumTeleOp : LinearOpMode() {

    // Instance of the "Robot" class
    private val robot = Robot(this)

    override fun runOpMode() {
        // By setting these values to new Gamepad(), they will default to all
        // boolean values as false and all float values as 0
        val currentGamepad1 = Gamepad()
        val previousGamepad1 = Gamepad()

        robot.initialize()

        /* Send telemetry message to signify robot waiting */
        telemetry.addLine("Robot Ready")
        telemetry.update()

        waitForStart()

        if (isStopRequested) return

        if (isStarted) {
            robot.moveIntakeSlideIn()
            robot.moveArm(ArmState.Transfer)
            robot.moveBucket(BucketState.UP)
            telemetry.addLine("Robot Started")
            telemetry.update()
        }

        while (opModeIsActive()) {
            // Store the gamepad values from the previous loop iteration in
            // previousGamepad1/2 to be used in this loop iteration.
            // This is equivalent to doing this at the end of the previous
            // loop iteration, as it will run in the same order except for
            // the first/last iteration of the loop.
            previousGamepad1.copy(currentGamepad1)

            // Store the gamepad values from this loop iteration in
            // currentGamepad1/2 to be used for the entirety of this loop iteration.
            // This prevents the gamepad values from changing between being
            // used and stored in previousGamepad1/2.
            currentGamepad1.copy(gamepad1)

            // START MECANUM DRIVE
            val armIntakeDown = robot.isArmIntakeDown()
            val powerReducer: Double = if (armIntakeDown) 1.0 else 0.0
            robot.drive(
                axial = -currentGamepad1.left_stick_y.toDouble(), // Remember, Y stick value is reversed
                lateral = currentGamepad1.left_stick_x.toDouble() * 1.1,
                yaw = currentGamepad1.right_stick_x.toDouble(),
                powerMultiplier = currentGamepad1.left_trigger.toDouble(),
                powerReducer = if (armIntakeDown) powerReducer else currentGamepad1.right_trigger.toDouble()
            )
            // END MECANUM DRIVE

            // START MOVE OUTTAKE SLIDE
            when {
                currentGamepad1.dpad_up -> robot.moveOuttakeSlide(OuttakeSlideState.ScoringInHighBasket)
                currentGamepad1.dpad_down -> robot.moveOuttakeSlide(OuttakeSlideState.Collapsed)
                currentGamepad1.guide -> robot.moveOuttakeSlide(OuttakeSlideState.LevelOneAscent)
                else -> robot.resetOuttakeSlideIfNotBusy() // SHOULD THIS BE CALLED EACH TIME???
            }
            // END MOVE OUTTAKE SLIDE

            // START MOVE ARM MOTOR
            if (currentGamepad1.triangle) {
                robot.moveArm(ArmState.IntakePickup)
            } else if (currentGamepad1.cross) {
                robot.moveArm(ArmState.Transfer)
            } else if (currentGamepad1.circle) {
                robot.moveArm(ArmState.LowChamberScoring)
            }
            // END MOVE ARM MOTOR

            // START MOVE BUCKET
            if (currentGamepad1.square && !previousGamepad1.square) {
                // This will set intakeToggle to true if it was previously false
                // and intakeToggle to false if it was previously true,
                // providing a toggling behavior.
                robot.moveBucket()
            }
            // END MOVE BUCKET

            // START MOVE INTAKE SLIDE
            if (currentGamepad1.dpad_left) {
                robot.moveIntakeSlideOut()
            } else if (currentGamepad1.dpad_right) {
                robot.moveIntakeSlideIn()
            }
            // END MOVE INTAKE SLIDE

            // START SPIN INTAKE
            if (currentGamepad1.left_bumper) {
                robot.spinIntakeIn()
            } else if (currentGamepad1.right_bumper) {
                robot.spinIntakeOut()
            } else {
                robot.stopSpinningIntake()
            }
            // END SPIN INTAKE

            // UPDATE LED COLORS
            robot.updateLedColors()

            // UPDATE ROBOT STATE
            robot.update()

            // PERFORM AUTOMATIONS
            robot.performAutomations()

            // UPDATE TELEMETRY DATA
            robot.logTelemetryData()
            telemetry.update()
        }
    }
}