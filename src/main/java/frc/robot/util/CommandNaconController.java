// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Add your docs here. */
public class CommandNaconController extends CommandGenericHID{

    public CommandNaconController(int port){
        super(port);
    }

    /** Represents a digital button on a PS4Controller. */
    public enum Button {
        /** Square button. */
        kSquare(3),
        /** X button. */
        kCross(1),
        /** Circle button. */
        kCircle(2),
        /** Triangle button. */
        kTriangle(4),
        /** Left Trigger 1 button. */
        kL1(5),
        /** Right Trigger 1 button. */
        kR1(6),
        /** Left Trigger 2 button. */
        //kL2(7),
        /** Right Trigger 2 button. */
        //kR2(8),
        /** Share button. */
        kShare(7),
        /** Option button. */
        kOptions(8),
        /** Left stick button. */
        kL3(9),
        /** Right stick button. */
        kR3(10);
        
        /** PlayStation button. */
        //kPS(13),
        /** Touchpad click button. */
        //kTouchpad(14);
        

        /** Button value. */
        public final int value;

        Button(int index) {
        this.value = index;
        }

        /**
         * Get the human-friendly name of the button, matching the relevant methods. This is done by
         * stripping the leading `k`, and if not the touchpad append `Button`.
         *
         * <p>Primarily used for automated unit tests.
         *
         * @return the human-friendly name of the button.
         */
        @Override
        public String toString() {
        var name = this.name().substring(1); // Remove leading `k`
        return name + "Button";
        }
    }

    /** Represents an axis on a PS4Controller. */
    public enum Axis {
        /** Left X axis. */
        kLeftX(0),
        /** Left Y axis. */
        kLeftY(1),
        /** Right X axis. */
        kRightX(4),
        /** Right Y axis. */
        kRightY(5),
        /** Left Trigger 2. */
        kL2(2),
        /** Right Trigger 2. */
        kR2(3);

        /** Axis value. */
        public final int value;

        Axis(int index) {
        value = index;
        }

        /**
         * Get the human-friendly name of the axis, matching the relevant methods. This is done by
         * stripping the leading `k`, and if one of L2/R2 append `Axis`.
         *
         * <p>Primarily used for automated unit tests.
         *
         * @return the human-friendly name of the axis.
         */
        @Override
        public String toString() {
        var name = this.name().substring(1); // Remove leading `k`
        if (name.endsWith("2")) {
            return name + "Axis";
        }
        return name;
        }
    }


    public double getRightX(){
        return this.getRawAxis(Axis.kRightX.value);
    }

    public double getRightY(){
        return this.getRawAxis(Axis.kRightY.value);
    }

    public double getLeftX(){
        return this.getRawAxis(Axis.kLeftX.value);
    }

    public double getLeftY(){
        return this.getRawAxis(Axis.kLeftY.value);
    }

    public double getR2Axis(){
        return this.getRawAxis(Axis.kR2.value);
    }

    public double getL2Axis(){
        return this.getRawAxis(Axis.kL2.value);
    }



    public Trigger cross(){
        return this.button(Button.kCross.value);
    }

    public Trigger circle(){
        return this.button(Button.kCircle.value);
    }

    public Trigger square(){
        return this.button(Button.kSquare.value);
    }

    public Trigger triangle(){
        return this.button(Button.kTriangle.value);
    }

    public Trigger R1(){
        return this.button(Button.kR1.value);
    }

    public Trigger L1(){
        return this.button(Button.kL1.value);
    }

    public Trigger R3(){
        return this.button(Button.kR3.value);
    }

    public Trigger L3(){
        return this.button(Button.kL3.value);
    }

    public Trigger share(){
        return this.button(Button.kShare.value);
    }

    public Trigger options(){
        return this.button(Button.kOptions.value);
    }
}
