package frc.controller;

import edu.wpi.first.wpilibj.Timer;

public class ButtonDebouncer {
    Ps4_Controller joystick;
    int buttonnum;
    double latest;
    double debounce_period;

    public ButtonDebouncer(Ps4_Controller joystick, int buttonnum ){ 
        this.joystick = joystick;
        this.buttonnum = buttonnum;
        this.latest = 0;
        this.debounce_period = .5; 
    }
    public ButtonDebouncer(Ps4_Controller joystick, int buttonnum, float period) { 
        this.joystick = joystick;
        this.buttonnum = buttonnum;
        this.latest = 0;
        this.debounce_period = period; 
    }
    public void setDebouncePeriod(float period)
    { 
        this.debounce_period = period;
    }
    public boolean get() {
        double now = Timer.getFPGATimestamp(); 
        if(joystick.getRawButton(buttonnum)){
            if( (now-latest) > debounce_period){ 
                latest = now;
                return true; 
            }
        }
    return false; 
    }
}