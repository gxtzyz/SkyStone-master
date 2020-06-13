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


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

import static java.lang.Thread.sleep;


@Autonomous(name="LoaderAutonomousRedRight", group="game1920")
public class LoaderAutonomousRedRight extends OpMode {
    public static final String TAG = LoaderAutonomousRedRight.class.getSimpleName();

    double RED_BLUE = -1.0;

    DriveTrainTeleOp driveTrainTeleOp;
    ArmHandTeleOp armHandTeleOp;
    private int loopCount = 0;

    ColorSensor colorSensor;
    private int scanCount = 0;

    public LoaderAutonomousRedRight() {
        super();
    }

    public LoaderAutonomousRedRight(OpMode parent) {
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
        RED_BLUE = 1.0;
    }

    @Override
    public void init() {
        driveTrainTeleOp = new DriveTrainTeleOp(this);
        driveTrainTeleOp.init();

        armHandTeleOp = new ArmHandTeleOp(this);
        armHandTeleOp.init();

        colorSensor = hardwareMap.get(ColorSensor.class, "HandColor");
        DETECT_RED = colorSensor.red();
        DETECT_GREEN = colorSensor.green();
        DETECT_BLUE = colorSensor.blue();
        telemetry.addData(TAG,"BASE: R(%7d), G(%7d), B(%7d)",
                DETECT_RED, DETECT_GREEN, DETECT_BLUE);
    }

    @Override
    public void loop() {
        switch (loopCount) {
            case 0: // move forward to stones
                driveTrainTeleOp.moveLiner(0.0, 0.6, 530
                );
                break;
            case 10: // flip arm out half way
                armHandTeleOp.inOut(-0.7, 2000);
                break;
            case 12: // open hand
                armHandTeleOp.openClose(1.0, 1200);
                break;
            case 18: // wrap a stone
                armHandTeleOp.inOut(-0.6, 1200);
                driveTrainTeleOp.moveLiner(0.0, 0.5, 100);
                break;
            case 20:
            case 21:
            case 22:
            case 23:
            case 24:
                if (isBlack()) { // detected skystone
                    loopCount = 30;
                } else {
                    // adjust turning
                    // move 8 inch left(red) / right(blue) to next stone
                    for (int il = 0; il < 3; il++) {
                        driveTrainTeleOp.moveLiner(0.4 * RED_BLUE, 0.0, 300);
                        driveTrainTeleOp.turnBig(-0.4 * RED_BLUE, 150);
                    }
                    // repeat scanning
                    scanCount++;
                }
            // moving along stones about 1 inch
                break;

            case 32: // capture a stone
                armHandTeleOp.openClose(-1.0, 1300);
                break;
            case 34: // flip in a little bit
                armHandTeleOp.inOut(0.7, 600);
                break;
            case 40: // turn toward building zone, red(right) and blue(left)
                driveTrainTeleOp.turn(0.7 * RED_BLUE, 1000);
                driveTrainTeleOp.moveLiner(0.7, 0.0, 400);
                break;
            case 50: // lower arm to pass gate
                armHandTeleOp.inOut(-0.6, 400);
                break;
            case 52: // move forward to building zone
                driveTrainTeleOp.moveLiner(0.0, 1.0, 1000 + scanCount * 150);
                break;
            case 62: // drop stone
                armHandTeleOp.openClose(1.0,800);
                break;
            case 70: // backoff for lift arm without tipping stone
                driveTrainTeleOp.moveLiner(0.0, -0.4, 200);
                break;
            case 80: // lift in to avoid graping stone back
                armHandTeleOp.inOut(0.7, 700);
                break;
            case 90: // move backward to loading zone
                driveTrainTeleOp.moveLiner(0.0, -0.6, 300);
                break;
//            case 100: // lower arm to avoid gate
//                armHandTeleOp.inOut(-0.7, 600);
//                break;
//            case 110:
//                driveTrainTeleOp.moveLiner(0.0, -1.0, 500 + scanCount * 150);
//                break;
//            case 120: // flip in/up arm to avoid knocking stones
//                armHandTeleOp.inOut(0.7, 600);
//                break;
//            case 130:
//                driveTrainTeleOp.turn(-0.7 * RED_BLUE, 1200);
            default:
                break;
        }
        loopCount++;
    }


    private int DETECT_RED = 150;
    private int DETECT_GREEN = 250;
    private int DETECT_BLUE = 150;
    private boolean isBlack() {
//       // int red = colorSensor.red();
//        //int green = colorSensor.green();
//       // int blue = colorSensor.blue();
//        telemetry.addData(TAG,"(%7d): R(%7d), G(%7d), B(%7d)", scanCount,
//                red, green, blue);
//        // empty space
//        int base = DETECT_RED + DETECT_GREEN + DETECT_BLUE;
//        int bright = red + green + blue;
//        //if (bright * 110 / 100 < base) { // empty
//           // return false;
//       // }
//        int strongBlue = blue * 150 / 100;
//        if (red > strongBlue && green > strongBlue ) { // yellow
//            return false;
//        }
//        // must be black
        return true;
    }
}