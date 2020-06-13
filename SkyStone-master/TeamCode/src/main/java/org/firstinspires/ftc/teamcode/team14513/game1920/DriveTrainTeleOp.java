/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.firstinspires.ftc.teamcode.team14513.game1920;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import static java.lang.Thread.sleep;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="DriveTrainTeleOp", group="game1920")
//@Disabled
public class DriveTrainTeleOp extends OpMode
{
    public static final String TAG = DriveTrainTeleOp.class.getSimpleName();

    public static final double GENERAL_MIN = 0.01;
    public static final long DEFAULT_RUN = 100;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftRearDrive = null;
    private DcMotor rightRearDrive = null;

    public DriveTrainTeleOp() {
        super();
    }

    public DriveTrainTeleOp(OpMode parent) {
        super();
        super.gamepad1 = parent.gamepad1;
        super.gamepad2 = parent.gamepad2;
        super.hardwareMap = parent.hardwareMap;
        super.internalOpModeServices = parent.internalOpModeServices;
        super.msStuckDetectInit = parent.msStuckDetectInit;
        super.msStuckDetectInitLoop = parent.msStuckDetectInitLoop;
        super.msStuckDetectLoop = parent.msStuckDetectLoop;
        super.msStuckDetectStart = parent.msStuckDetectStart;
        super.msStuckDetectStop = parent.msStuckDetectStop;
        super.telemetry = parent.telemetry;
        super.time = parent.time;
    }

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "FrontLeft");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "FrontRight");
        leftRearDrive  = hardwareMap.get(DcMotor.class, "RearLeft");
        rightRearDrive = hardwareMap.get(DcMotor.class, "RearRight");

        leftFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        leftRearDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRearDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        // Tell the driver that initialization is complete.
        telemetry.addData(TAG, "Initialized");
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        realLoop(gamepad1, "G1> ");

        telemetry.addData(TAG, "Run Time: " + runtime.toString());
        telemetry.update();
    }

    private boolean realLoop(Gamepad gp, String gpid) {
        telemetry.addData(TAG, gpid + "lx(%.2f), ly(%.2f)", gp.left_stick_x, gp.left_stick_y);
        telemetry.addData(TAG, gpid + "lt(%.2f)", gp.left_trigger);
        telemetry.addData(TAG, gpid + "rt(%.2f)", gp.right_trigger);

        if (Math.abs(gp.left_stick_x) > GENERAL_MIN || Math.abs(gp.left_stick_y) > GENERAL_MIN) {
            moveLiner(gp.left_stick_x, -gp.left_stick_y, DEFAULT_RUN);
            return true;
        }
        if (Math.abs(gp.left_trigger) > GENERAL_MIN) {
            turn(gp.left_trigger, DEFAULT_RUN);
            return true;
        }
        if (Math.abs(gp.right_trigger) > GENERAL_MIN) {
            turn(-gp.right_trigger, DEFAULT_RUN);
            return true;
        }
        if (gp.left_bumper) {
            turnBig(0.5, DEFAULT_RUN);
            return true;
        }
        if (gp.right_bumper) {
            turnBig(-0.5, DEFAULT_RUN);
            return true;
        }
        if (gp.dpad_up) {
            moveLiner(0.0, 0.5, DEFAULT_RUN);
            return true;
        }
        if (gp.dpad_down) {
            moveLiner(0.0, -0.5, DEFAULT_RUN);
            return true;
        }
        if (gp.dpad_left) {
            moveLiner(-0.5, 0.0, DEFAULT_RUN);
            return true;
        }
        if (gp.dpad_right) {
            moveLiner(0.5, 0.0, DEFAULT_RUN);
            return true;
        }
        return false;
    }

    void moveLiner(double xx, double yy, long runTime) {
        double x = xx;
        double y = yy * 0.7;
        leftFrontDrive.setPower((x - y));
        rightFrontDrive.setPower((x + y));
        // weight center is at front, rear side need less power to balance
        leftRearDrive.setPower(-(x + y));
        rightRearDrive.setPower((y - x));

        try {
            sleep(runTime);
        } catch (InterruptedException e) {
            e.printStackTrace();
        } finally {
            leftFrontDrive.setPower(0.0);
            rightFrontDrive.setPower(0.0);
            leftRearDrive.setPower(0.0);
            rightRearDrive.setPower(0.0);
        }
    }

    void turn(double tv, long runTime) {
        leftFrontDrive.setPower(tv);
        rightFrontDrive.setPower(tv);
        leftRearDrive.setPower(tv);
        rightRearDrive.setPower(tv);

        try {
            sleep(runTime);
        } catch (InterruptedException e) {
            e.printStackTrace();
        } finally {
            leftFrontDrive.setPower(0.0);
            rightFrontDrive.setPower(0.0);
            leftRearDrive.setPower(0.0);
            rightRearDrive.setPower(0.0);
        }
    }

    void turnBig(double tv, long runTime) {
        double weak = 0.5;
        if (tv > 0.0) {
            leftFrontDrive.setPower(tv);
            leftRearDrive.setPower(tv);
            rightFrontDrive.setPower(tv * weak);
            rightRearDrive.setPower(tv * weak);
        } else {
            rightFrontDrive.setPower(tv);
            rightRearDrive.setPower(tv);
            leftFrontDrive.setPower(tv * weak);
            leftRearDrive.setPower(tv * weak);
        }
        try {
            sleep(runTime);
        } catch (InterruptedException e) {
            e.printStackTrace();
        } finally {
            leftFrontDrive.setPower(0.0);
            rightFrontDrive.setPower(0.0);
            leftRearDrive.setPower(0.0);
            rightRearDrive.setPower(0.0);
        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
