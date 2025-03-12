// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import javax.swing.JFrame;

public class Keyboard extends JFrame implements KeyListener {

    public static boolean isRight = false;
    public static int level = 0;
    private static boolean leftArrowPressed = false;
    private static boolean rightArrowPressed = false;
    private static boolean oneKeyPressed = false;
    private static boolean twoKeyPressed = false;
    private static boolean threeKeyPressed = false;
    private static boolean fourKeyPressed = false;

    public Keyboard() {
        this.setTitle("Keyboard Input for WPILib");
        this.setSize(200, 200);
        this.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        this.addKeyListener(this);
        this.setVisible(true);
    }

    @Override
    public void keyPressed(KeyEvent e) {
        if (e.getKeyCode() == KeyEvent.VK_LEFT) {
            leftArrowPressed = true;
        } else if (e.getKeyCode() == KeyEvent.VK_RIGHT) {
            rightArrowPressed = true;
            
        }
    }
    
    
    

    @Override
    public void keyReleased(KeyEvent e) {
        if (e.getKeyCode() == KeyEvent.VK_LEFT) {
            leftArrowPressed = false;

        } else if (e.getKeyCode() == KeyEvent.VK_RIGHT) {
            rightArrowPressed = false;
          
        }
    }

    public static boolean isLeftArrowPressed() {
        return leftArrowPressed;
    }

    // Boolean method to check if right arrow is pressed
    public static boolean isRightArrowPressed() {
        return rightArrowPressed;
    }

    public static boolean is1KeyPressed() {
        return oneKeyPressed;
    }

    // Boolean method to check if right arrow is pressed
    public static boolean is2KeyPressed() {
        return twoKeyPressed;
    }

    public static boolean is3KeyPressed() {
        return threeKeyPressed;
    }

    public static boolean is4KeyPressed() {
        return fourKeyPressed;
    }



    @Override
    public void keyTyped(KeyEvent e) {}
}


