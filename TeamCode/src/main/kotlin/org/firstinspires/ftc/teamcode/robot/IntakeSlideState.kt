package org.firstinspires.ftc.teamcode.robot

private const val INTAKE_SLIDE_SERVO_START_POSITION = 0.0
private const val INTAKE_SLIDE_SERVO_END_POSITION = 0.28

enum class IntakeSlideState(val position: Double) {
    IN(INTAKE_SLIDE_SERVO_START_POSITION),
    OUT(INTAKE_SLIDE_SERVO_END_POSITION),
}