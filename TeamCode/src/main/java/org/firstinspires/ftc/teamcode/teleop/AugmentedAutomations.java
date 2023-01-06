package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name = "Augmented Automations", group = "Alpha")
public class AugmentedAutomations extends LinearOpMode {
    SampleMecanumDrive drive;

    DcMotor liftL = null;
    DcMotor liftR = null;
    DcMotor liftT = null;

    Servo claw = null;
    Servo extend = null;
    Servo tilt = null;
    Servo retraction = null;
    Servo camera = null;

    DcMotor turret = null;

    Gamepad g1 = gamepad1;
    Gamepad g2 = gamepad2;

    double extensionPos = Constants.extendInPos;
    double extensionRange = Constants.extendOutPos - Constants.extendInPos;
    double cameraPos = Constants.cameraDownPos;

    double tiltPos = Constants.tiltDownPosition;
    long timerStart = System.currentTimeMillis();
    long timer2 = System.currentTimeMillis();
    long timer3 = System.currentTimeMillis();
    long loopTimer = System.currentTimeMillis();

    boolean isHolding = false;
    boolean isOpen = true;
    boolean isMoving = false;
    boolean hasPressed = false;
    boolean hasPulled = false;
    boolean hasTilted = false;
    boolean isDelaying = false;
    boolean finishedDeposit = false;
    boolean isDepositing = false;

    int liftTarget = 0;
    double turretStorage = 0;
    int liftStorage = 0;

    int highTarget = Constants.liftTargetHighTeleop;
    int midTarget = Constants.liftTargetMid;
    int lowTarget = Constants.liftTargetLow;

    int resetStatus = 0;
    int depositStatus = 0;

    int holdPos = 0;

    double driveMultiplier = 0.75;

    @Override
    public void runOpMode() throws InterruptedException {
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        g1 = gamepad1; //Defining gs Aarav Mehta
        g2 = gamepad2;

        Constants.initHardware(hardwareMap);

        liftL = Constants.liftL;
        liftR = Constants.liftR;
        liftT = Constants.liftT;

        claw = Constants.claw;
        extend = Constants.extend;
        tilt = Constants.tilt;
        retraction = Constants.retraction;
        camera = Constants.camera;

        turret = Constants.turret;

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        camera.setPosition(Constants.cameraDownPos);

        Constants.setClaw(Constants.ClawPosition.OPEN);
        retraction.setPosition(Constants.retractionUpPos);
        extend.setPosition(Constants.extendInPos - 0.3);
        extensionPos = Constants.extendInPos - 0.3;

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            turretAuto();
            teleopCode();
            teleopAuto();
        }
    }

    public void liftAuto() {
        if (g2.right_trigger == 1 && g2.left_trigger == 1) {
            finishedDeposit = true;
            if ((liftR.getCurrentPosition()) > Constants.liftTargetLow + (Constants.turretError + 100)) {
                liftTarget = Constants.liftTargetLow;
                depositStatus = 1;
            } else
                depositStatus = 2;

            if ((g2.dpad_up)) {
                liftTarget = highTarget;
            } else if ((g2.dpad_left)) {
                liftTarget = midTarget;
            } else if ((g2.dpad_right)) {
                liftTarget = lowTarget;
            } else if (g2.dpad_down) {
                liftTarget = Constants.liftTargetLow;
            }

            if (Math.abs(liftR.getCurrentPosition() - liftTarget) >= Constants.liftError)
                Constants.setLift(liftTarget, Constants.liftPower);
        } else {
            if (depositStatus > 0) {
                if (!isDelaying) {
                    isDelaying = true;
                    timer3 = System.currentTimeMillis();
                } else if (System.currentTimeMillis() - timer3 >= 400) {
                    depositStatus = 0;
                    isDelaying = false;
                }
            } else {
                if (Math.abs(g2.left_stick_x) <= 0.2) {
                    if ((g2.dpad_up) && Math.abs(liftR.getCurrentPosition() - highTarget) >= Constants.liftError) {
                        Constants.setLift(highTarget, Constants.liftPower);
                    }
                    if ((g2.dpad_left) && Math.abs(liftR.getCurrentPosition() - midTarget) >= Constants.liftError) {
                        Constants.setLift(midTarget, Constants.liftPower);
                    }
                    if ((g2.dpad_right) && Math.abs(liftR.getCurrentPosition() - lowTarget) >= Constants.liftError) {
                        Constants.setLift(lowTarget, Constants.liftPower);
                    }
                    if ((g2.dpad_down) && Math.abs(liftR.getCurrentPosition()) >= Constants.liftError && Math.abs(turret.getCurrentPosition()) >= Constants.turretError) {
                        Constants.setLift(-700, Constants.liftPower);
                        finishedDeposit = false;
                        resetStatus = 1;
                    } else if ((g2.dpad_down) && Math.abs(liftR.getCurrentPosition()) >= Constants.liftError) {
                        Constants.setLift(0, Constants.liftPower);
                        finishedDeposit = false;
                        resetStatus = 2;
                    }
                }

                if (Math.abs(g2.left_stick_x) > 0.2) {
                    if (!isHolding) {
                        isHolding = true;
                        holdPos = liftR.getCurrentPosition();
                    }
                    Constants.setLift(holdPos, Constants.liftPower);
                }
                if ((!finishedDeposit || g2.left_stick_y != 0) && !g2.dpad_down && !g2.dpad_up && !g2.dpad_right && !g2.dpad_left && Math.abs(g2.left_stick_x) <= 0.2) {
                    liftL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    liftR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    liftT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                    finishedDeposit = false;

                    resetStatus = 0;
                    double liftInput = square(g2.left_stick_y) * Constants.liftPower;

                    isHolding = false;
                    holdPos = 0;

                    if (liftR.getCurrentPosition() > Constants.liftLimit || g2.left_stick_y < 0) {
                        liftL.setPower(-liftInput);
                        liftR.setPower(liftInput);
                        liftT.setPower(-liftInput);
                    }
                }
            }
        }
    }

    public void turretAuto() {
        if (depositStatus == 0) {
            double turretMultiplier = 0.5;
            if (g2.dpad_down && Math.abs(turret.getCurrentPosition()) >= Constants.liftError) {
                Constants.setTurret(0, false, turretMultiplier);
            }
            if (!g2.dpad_down) {
                turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                double triggerDiff = g2.right_trigger - g2.left_trigger;
                double turretInput = -square(triggerDiff) * turretMultiplier;

                turret.setPower(turretInput);
            }
        } else if (depositStatus == 2) {
            double turretMultiplierAuto = 0.7;
            double y = -g2.left_stick_y;
            double x = g2.left_stick_x;

            if (g2.left_stick_button) {
                Constants.setTurret((int) (turretStorage), true, 1);
            } else if (Math.pow(x, 2) + Math.pow(y, 2) > 0.9) {
                double angle = Math.toDegrees(Math.atan2(y, x)) / 180;
                if (angle < -0.5 && angle >= -1) {
                    angle = -1 - angle;
                    angle = angle - 0.5;
                    angle = Math.abs(angle);
                } else {
                    angle -= 0.5;
                }

                if (turret.getCurrentPosition() > 700 && angle <= -0.5)
                    angle += 2;
                if (turret.getCurrentPosition() < -700 && angle >= 0.5)
                    angle -= 2;

                turretStorage = 1400 * angle;

                if (Math.abs(turret.getCurrentPosition() - turretStorage) >= Constants.liftError)
                    Constants.setTurret((int) (1400 * angle), true, turretMultiplierAuto);
//                double rawAngle = Math.acos(Math.abs(x));
//                if (Math.abs(x) < .3 && y > 0) // up
//                    Constants.setTurret(Constants.turretTarget180, true, turretMultiplierAuto);
//                else if (Math.abs(x) < .3) // down
//                    Constants.setTurret(0, true, turretMultiplierAuto);
//                else if (x > 0.92) // right
//                    Constants.setTurret(Constants.turretTarget90, true, turretMultiplierAuto);
//                else if (x < -0.92) // left
//                    Constants.setTurret(Constants.turretTargetNeg90, true, turretMultiplierAuto);
//                else if (x > 0 && y > 0) // top right
//                    Constants.setTurret(Constants.turretTargetAutonL, true, turretMultiplierAuto);
//                else if (x < 0 && y > 0) // top left
//                    Constants.setTurret(Constants.turretTargetAutonR, true, turretMultiplierAuto);
//                else if (x > 0 && y < 0) // bottom right
//                    Constants.setTurret(Constants.turretTargetTopRight, true, turretMultiplierAuto);
//                else if (x < 0 && y < 0) // bottom left
//                    Constants.setTurret(Constants.turretTargetTopLeft, true, turretMultiplierAuto);
            }
        }
    }

    public void resetAuto() {
        if (g2.b) {
            if (g2.left_stick_button) {
                highTarget = Constants.liftTargetHighTeleop;
                midTarget = Constants.liftTargetMid;
                lowTarget = Constants.liftTargetLow;
            } else if (Math.abs(g2.left_stick_x) > 0.2) {
                if (liftR.getCurrentPosition() < -1950)
                    highTarget = liftR.getCurrentPosition();
                else if (liftR.getCurrentPosition() > -1200)
                    midTarget = liftR.getCurrentPosition();
                else
                    lowTarget = liftR.getCurrentPosition();
            } else {
                liftL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                liftL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                liftR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                liftR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                liftT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                liftT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }
    }

    public void teleopAuto() {
        turretAuto();
        liftAuto();
        resetAuto();

    }

    public void armCode() {
        if (g2.right_stick_x > 0 && liftR.getCurrentPosition() >= -500) {
            extensionPos += -(g2.right_stick_x) * Constants.extendSensitivity;
        } else if (g2.right_stick_x > 0) {
            extensionPos += -(g2.right_stick_x) * Constants.extendSensitivity * 5;
        } else {
            extensionPos += -(g2.right_stick_x) * Constants.extendSensitivity;
        }


        if (extensionPos < Constants.extendOutPos)
            extensionPos = Constants.extendOutPos;
        if (extensionPos > Constants.extendInPos)
            extensionPos = Constants.extendInPos;

        if (g2.right_stick_button)
            extensionPos = Constants.extendInPos;

        if (resetStatus == 1) {
            extensionPos = Constants.extendInPos - 0.1;
            hasPulled = false;
        }
        if (resetStatus == 2) {
            extensionPos = Constants.extendInPos - 0.3;
            hasPulled = false;
        }

        if (depositStatus > 0) {
            if (liftR.getCurrentPosition() > -200) {
                extensionPos = Constants.extendInPos - 0.3;
                hasPulled = false;
            } else {
                extensionPos = Constants.extendInPos;
                hasPulled = false;
            }
        }

        if (depositStatus == 0 && (g2.dpad_up || g2.dpad_left || g2.dpad_right) && !hasPulled) {
            extensionPos = Constants.extendInPos;
            hasPulled = true;
        }


        tiltPos += -(g2.right_stick_y) * Constants.tiltSensitivity;

        if (tiltPos > Constants.tiltUpPosition)
            tiltPos = Constants.tiltUpPosition;

        if (depositStatus == 0 && (g2.dpad_up || g2.dpad_left || g2.dpad_right) && !hasTilted) {
            tiltPos = Constants.tiltDropPosition;
            hasTilted = true;
        } else if (g2.dpad_down) {
            tiltPos = Constants.tiltDownPosition;
            hasTilted = false;
        }

        if (depositStatus == 0) {
            if (g2.y)
                tiltPos = Constants.tiltUpPosition;
            if (g2.x)
                tiltPos = Constants.tiltDropPosition;
            if (g2.a) {
                tiltPos = Constants.tiltDownPosition;
                hasTilted = false;
            }
        } else {
            tiltPos = Constants.tiltDropPosition;
        }


        if (g2.left_bumper && g2.right_stick_x == 0) {
            extensionPos = Constants.extendOutPos;
            isDepositing = true;
        } else if (g2.right_stick_x != 0) {
            isDepositing = false;
        }

        if (isDepositing && !g2.left_bumper) {
            turretStorage = turret.getCurrentPosition();
            tiltPos = Constants.tiltDownPosition;
            hasTilted = false;
            isDepositing = false;
            if (!isMoving) {
                timerStart = System.currentTimeMillis();
                isMoving = true;
            }

            if (liftR.getCurrentPosition() < -1950)
                highTarget = liftR.getCurrentPosition();
            else if (liftR.getCurrentPosition() > -1200)
                midTarget = liftR.getCurrentPosition();
            else
                lowTarget = liftR.getCurrentPosition();
        } else if (!(g2.left_bumper))
            isDepositing = false;

        if (isMoving && System.currentTimeMillis() - timerStart >= 300) {
            Constants.setClaw(Constants.ClawPosition.OPEN);
            if (liftR.getCurrentPosition() < -200)
                extensionPos = Constants.extendInPos;
            else
                extensionPos = Constants.extendInPos - 0.3;
            isMoving = false;
            finishedDeposit = false;
            isOpen = true;
        }

        if (g2.right_bumper) {
            if (!hasPressed) {
                timer2 = System.currentTimeMillis();
                hasPressed = true;

                if (isOpen)
                    Constants.setClaw(Constants.ClawPosition.CLOSED);
                else
                    Constants.setClaw(Constants.ClawPosition.OPEN);
                isOpen = !isOpen;
            }
        }
        if (System.currentTimeMillis() - timer2 >= 250) {
            hasPressed = false;
        }

        extend.setPosition(extensionPos);
        tilt.setPosition(tiltPos);

    }

    public void driveCode() {
        double y = -g1.left_stick_y;
        double x = -g1.left_stick_x;
        double turn = -g1.right_stick_x;

        double power = 2.0;

        if (liftR.getCurrentPosition() < -2000)
            driveMultiplier = 0.4;
        else
            driveMultiplier = 0.75;

        if (g1.right_trigger == 1) {
            driveMultiplier = 0.3;
        }
        if (g1.left_trigger == 1) {
            driveMultiplier = 1;
        }

        double yMult = driveMultiplier;
        double xMult = driveMultiplier;
        double turnMult = driveMultiplier;

        if (y < 0) yMult *= -1;
        if (x < 0) xMult *= -1;
        if (turn < 0) turnMult *= -1;

        y = Math.pow(y, power) * yMult;
        x = Math.pow(x, power) * xMult;
        turn = Math.pow(turn, power) * turnMult;

        drive.setWeightedDrivePower(new Pose2d(y, x, turn));

        if (g1.dpad_up)
            retraction.setPosition(Constants.retractionUpPos);
        if (g1.dpad_down)
            retraction.setPosition(Constants.retractionDownPos);

        if (g1.dpad_right)
            cameraPos += 0.01;
        if (g1.dpad_left)
            cameraPos -= 0.01;
        camera.setPosition(cameraPos);
    }

    public void telemetryCode() {
        telemetry.addData("Turret position", turret.getCurrentPosition());
        telemetry.addData("Lift position", liftR.getCurrentPosition());
        telemetry.addData("", "");
        telemetry.addData("Extension position", extend.getPosition());
        telemetry.addData("Tilt position", tilt.getPosition());
        telemetry.addData("", "");
        telemetry.addData("High target", highTarget);
        telemetry.addData("Mid target", midTarget);
        telemetry.addData("Low target", lowTarget);
        telemetry.addData("", "");
        telemetry.addData("Loop time", System.currentTimeMillis() - loopTimer);
        loopTimer = System.currentTimeMillis();

        telemetry.update();
    }

    public void teleopCode() {
        driveCode();
        armCode();
        telemetryCode();
    }

    public double square(double input) {
        double result = Math.pow(input, 2);
        if (input < 0) result *= -1;
        return result;
    }
}
