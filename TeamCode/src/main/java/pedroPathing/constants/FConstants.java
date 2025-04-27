package pedroPathing.constants;

import com.pedropathing.localization.Localizers;
import com.pedropathing.follower.FollowerConstants;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class FConstants {
    static {
        FollowerConstants.localizers = Localizers.PINPOINT;

        FollowerConstants.leftFrontMotorName = "frontLeftMotor";
        FollowerConstants.leftRearMotorName = "backLeftMotor";
        FollowerConstants.rightFrontMotorName = "frontRightMotor";
        FollowerConstants.rightRearMotorName = "backRightMotor";

        FollowerConstants.leftFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
        FollowerConstants.leftRearMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
        FollowerConstants.rightRearMotorDirection = DcMotorSimple.Direction.FORWARD;

        FollowerConstants.mass = 11.2;

        FollowerConstants.xMovement = 82.502272993526;
        FollowerConstants.yMovement = 67.042469589951;

        FollowerConstants.forwardZeroPowerAcceleration = -34.452164496463;
        FollowerConstants.lateralZeroPowerAcceleration = -64.90269827005;

        FollowerConstants.translationalPIDFCoefficients.setCoefficients(0.15,0,0.01,0);
        FollowerConstants.useSecondaryTranslationalPID = false;
        FollowerConstants.secondaryTranslationalPIDFCoefficients.setCoefficients(0.1,0,0.01,0); // Not being used, @see useSecondaryTranslationalPID

        FollowerConstants.headingPIDFCoefficients.setCoefficients(1.5,0,0.1,0);
        FollowerConstants.useSecondaryHeadingPID = false;
        FollowerConstants.secondaryHeadingPIDFCoefficients.setCoefficients(2,0,0.1,0); // Not being used, @see useSecondaryHeadingPID

        FollowerConstants.drivePIDFCoefficients.setCoefficients(0.010,0,0.000012,0.6,0);
        FollowerConstants.useSecondaryDrivePID = false;
        FollowerConstants.secondaryDrivePIDFCoefficients.setCoefficients(0.1,0,0,0.6,0); // Not being used, @see useSecondaryDrivePID

        FollowerConstants.zeroPowerAccelerationMultiplier = 1;
        FollowerConstants.centripetalScaling = 0.0003;

        FollowerConstants.pathEndTimeoutConstraint = 500;
        FollowerConstants.pathEndTValueConstraint = 0.995;
        FollowerConstants.pathEndVelocityConstraint = 0.1;
        FollowerConstants.pathEndTranslationalConstraint = 0.1;
        FollowerConstants.pathEndHeadingConstraint = 0.007;

// SEATTLE SOLVERS EXAMPLE CONSTANTS FOR SECONDARY PID TUNERS
//        FollowerConstants.translationalPIDFCoefficients.setCoefficients(0.16,0,0.01,0);
//        FollowerConstants.useSecondaryTranslationalPID = true;
//        FollowerConstants.secondaryTranslationalPIDFCoefficients.setCoefficients(0.13,0,0.015,0); // @see useSecondaryTranslationalPID
//
//        FollowerConstants.headingPIDFCoefficients.setCoefficients(1.5,0,0.1,0);
//        FollowerConstants.useSecondaryHeadingPID = true;
//        FollowerConstants.secondaryHeadingPIDFCoefficients.setCoefficients(0.9,0,0.08,0); // @see useSecondaryHeadingPID
//
//        FollowerConstants.drivePIDFCoefficients.setCoefficients(0.018,0,0.0000012,0.6,0);
//        FollowerConstants.useSecondaryDrivePID = true;
//        FollowerConstants.secondaryDrivePIDFCoefficients.setCoefficients(0.01,0,0.000001,0.6,0); // @see useSecondaryDrivePID
//
//        FollowerConstants.zeroPowerAccelerationMultiplier = 3;
//        FollowerConstants.centripetalScaling = 0.0004;
//
//        FollowerConstants.pathEndTimeoutConstraint = 50;
//        FollowerConstants.pathEndTValueConstraint = 0.98;
//        FollowerConstants.pathEndVelocityConstraint = 0.1;
//        FollowerConstants.pathEndTranslationalConstraint = 0.1;
//        FollowerConstants.pathEndHeadingConstraint = 0.006;
// SEATTLE SOLVERS EXAMPLE CONSTANTS FOR SECONDARY PID TUNERS
    }
}
