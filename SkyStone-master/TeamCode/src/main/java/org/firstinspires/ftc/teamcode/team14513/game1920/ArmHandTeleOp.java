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


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
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

@TeleOp(name="ArmHandTeleOp", group="game1920")
//@Disabled
public class ArmHandTeleOp extends OpMode
{
    public static final String TAG = ArmHandTeleOp.class.getSimpleName();

    public static final double UP_ABS_MAX = 1.0;
    public static final double DOWN_ABS_MAX = 0.4;
    public static final double IN_ABS_MAX = 0.7;
    public static final double OUT_ABS_MAX = 0.7;
    public static final double OPEN_ABS_MAX = 0.7;
    public static final double CLOSE_ABS_MAX = 0.4;
    public static final double GENERAL_ABS_MIN = 0.1;
    public static final long DEFAULT_RUN = 100;

    public static final double clip(double origin, double abs_cap) {
        double clip = origin * abs_cap;
        if (clip >= 0.0) {
            if (clip > abs_cap) {
                clip = abs_cap;
            }
            if (clip < GENERAL_ABS_MIN) {
                clip = GENERAL_ABS_MIN;
            }
        } else {
            if (clip < -abs_cap) {
                abs_cap = -abs_cap;
            }
            if (clip > -GENERAL_ABS_MIN) {
                clip = -GENERAL_ABS_MIN;
            }
        }
        return clip;
    }

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor armDrive = null;
    private DcMotor handDrive = null;
    private DcMotor flipDrive = null;

    private Servo lockerServo = null;
    private Servo markerServo = null;

    public ArmHandTeleOp() {
        super();
    }

    public ArmHandTeleOp(OpMode parent) {
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
        telemetry.addData("Status", "ArmHandTeleOp Initializing");

        armDrive  = hardwareMap.get(DcMotor.class, "Arm");
        handDrive = hardwareMap.get(DcMotor.class, "Hand");
        flipDrive  = hardwareMap.get(DcMotor.class, "Flip");

        lockerServo = hardwareMap.get(Servo.class, "Locker");
        markerServo = hardwareMap.get(Servo.class, "Marker");

        lockerServo.setPosition(-0.8);
        markerServo.setPosition(0.0);

        flipDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
        realLoop(gamepad2, "G2> ");

        telemetry.addData(TAG, "Run Time: " + runtime.toString());
    }

    private boolean realLoop(Gamepad gamepad, String padID) {
        telemetry.addData(TAG,padID + "ry=" + gamepad.right_stick_y);
        telemetry.addData(TAG,padID + "up=" + gamepad.dpad_up);
        telemetry.addData(TAG,padID + "down=" + gamepad.dpad_down);
        telemetry.addData(TAG,padID + "Y=" + gamepad.y);
        telemetry.addData(TAG,padID + "A=" + gamepad.a);

        if (Math.abs(gamepad.right_stick_y) > 0.0) {
            inOut(gamepad.right_stick_y, DEFAULT_RUN / 2);
            return true;
        }
        if (gamepad.dpad_up) {
            upDown(-1.0, DEFAULT_RUN);
            return true;
        }
        if (gamepad.dpad_down) {
            upDown(0.5, DEFAULT_RUN);
            return true;
        }
        if (gamepad.y) {
            openClose(0.8, DEFAULT_RUN);
            return true;
        }
        if (gamepad.a) {
            openClose(-0.5, DEFAULT_RUN);
            return true;
        }
        if (gamepad.x) {
            lock();
            return true;
        }
        if (gamepad.b) {
            unlock();
            return true;
        }
        if (gamepad.left_stick_button) {
            dropMaker();
            return true;
        }
        if (gamepad.right_stick_button) {
            resetMarker();
        }
        return false;
    }


    void upDown(double howMuch, long runTime) {
        if (howMuch >= 0.0) { // up
            armDrive.setPower(clip(howMuch, UP_ABS_MAX));
        } else { // down slow
            armDrive.setPower(clip(howMuch, DOWN_ABS_MAX));
        }

        try {
            sleep(runTime);
        } catch (InterruptedException e) {
            e.printStackTrace();
        } finally {
            armDrive.setPower(0.0);
        }
    }

    void inOut(double howMuch, long runTime) {
        if (howMuch >= 0.05) { // in
            flipDrive.setPower(clip(howMuch, IN_ABS_MAX));
        } else if (howMuch <= -0.05){ // out
            flipDrive.setPower(clip(howMuch, OUT_ABS_MAX));
        } else {
            flipDrive.setPower(0.0);
        }

        try {
            sleep(runTime);
        } catch (InterruptedException e) {
            e.printStackTrace();
        } finally {
            flipDrive.setPower(0.0);
        }
    }

    void openClose(double howMuch, long runTime) {
        if (howMuch >= 0.0) { // open
            handDrive.setPower(clip(howMuch, OPEN_ABS_MAX));
        } else { // close slow
            handDrive.setPower(clip(howMuch, CLOSE_ABS_MAX));
        }

        try {
            sleep(runTime);
        } catch (InterruptedException e) {
            e.printStackTrace();
        } finally {
            handDrive.setPower(0.0);
        }
    }

    void lock() {
        lockerServo.setDirection(Servo.Direction.FORWARD);
        lockerServo.setPosition(1.0);
    }

    void unlock() {
        lockerServo.setDirection(Servo.Direction.FORWARD);
        lockerServo.setPosition(-1.0);
    }

    void dropMaker() {
        markerServo.setDirection(Servo.Direction.FORWARD);
        markerServo.setPosition(0.5);
    }

    void resetMarker() {
        markerServo.setDirection(Servo.Direction.FORWARD);
        markerServo.setPosition(0.0);
    }


    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
