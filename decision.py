import numpy as np


def move_forward(Rover):
    steering_angle = 15 if Rover.vel < 1.5 else 12
    angles_in_degrees = Rover.nav_angles * 180/np.pi
    steer = np.mean(angles_in_degrees) #if Rover.nav_dists[np.argmax(Rover.nav_angles)] < 20 \
            #    or Rover.nav_dists[np.argmax(Rover.nav_angles)] > 80 \
            #else np.max(angles_in_degrees)
    
    # Check the extent of navigable terrain
    if len(Rover.nav_angles) >= Rover.stop_forward:  
        # If mode is forward, navigable terrain looks good 
        # and velocity is below max, then throttle 
        if Rover.vel < Rover.max_vel:
            print ('THROTTLING...')
            # Set throttle value to throttle setting
            Rover.throttle = Rover.throttle_set
        elif Rover.vel > Rover.max_vel: # Else coast
            print ('COASTING...')
            Rover.throttle = 0
            
        if Rover.vel == 0 and Rover.throttle > 0:
            print ('STUCK...')
            Rover.throttle = 0
            Rover.mode = 'reverse'
            
        if Rover.roll > 10 and Rover.vel > 1:
            print ('TOO FAST AND TOO MUCH ROLL...GETTING OF THE GAS...')
            Rover.throttle = 0
            if Rover.vel > 1.5:
                print ('WAY TOO FAST, BRAKING...')
                Rover.brake = 0.15
            
        Rover.brake = 0
        # Set steering to average angle clipped to the range +/- 15
        Rover.steer = np.clip(steer, -steering_angle, steering_angle) 
    # If there's a lack of navigable terrain pixels then go to 'stop' mode
    elif len(Rover.nav_angles) < Rover.stop_forward:
            print ('FORWARD TO STOP...')
            # Set mode to "stop" and hit the brakes!
            Rover.throttle = 0
            # Set brake to stored brake value
            Rover.brake = Rover.brake_set
            Rover.steer = 0
            Rover.mode = 'stop'
            
    if np.mean(Rover.nav_dists) < 20:
        print ('LIMITED DISTANCE FORWARD')
        if Rover.vel > 1:
            print ('LIMITED VIEW FORWARD: BRAKING...')
            Rover.throttle = 0
            Rover.brake = 0.1
        else:
            print ('LIMITED VIEW FORWARD: COASTING...')
            Rover.throttle = 0
            
    if Rover.near_sample: 
        print ('NEAR SAMPLE: STOPPED')
        Rover.throttle = 0
        Rover.brake = 100
        Rover.mode = 'stop'
        

def stop_moving(Rover):
    # If we're in stop mode but still moving keep braking
    if Rover.vel > 0.2:
        print ('STOPPING...')
        Rover.throttle = 0
        Rover.brake = Rover.brake_set
        Rover.steer = 0
    # If we're not moving (vel < 0.2) then do something else
    elif Rover.vel <= 0.2:
        #if Rover.near_sample: 
        # Now we're stopped and we have vision data to see if there's a path forward
        if len(Rover.nav_angles) < Rover.go_forward:
            print ('TURNING...')
            Rover.throttle = 0
            # Release the brake to allow turning
            Rover.brake = 0
            # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
            Rover.steer = -15 # Could be more clever here about which way to turn
            Rover.mode = 'stop'
        # If we're stopped but see sufficient navigable terrain in front then go!
        if len(Rover.nav_angles) >= Rover.go_forward:
            print ('STOP TO FORWARD')
            # Set throttle back to stored value
            Rover.throttle = Rover.throttle_set
            # Release the brake
            Rover.brake = 0
            # Set steer to mean angle
            Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
            Rover.mode = 'forward'
      
           
def move_reverse(Rover, distance_thresh=50):
    print ('REVERSE')
    Rover.throttle = -0.1
    Rover.brake = 0
    Rover.steer = 0
    if np.mean(Rover.nav_dists) > distance_thresh and len(Rover.nav_angles) > 500:
        print ('REVERSE TO STOP')
        Rover.throttle = 0
        Rover.brake = 1
        Rover.mode = 'stop'
    else: 
        print ('STILL STUCK, KEEP REVERSING')
        Rover.mode = 'reverse'
    

# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):

    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!
    
    # Example:
    # Check if we have vision data to make decisions with
    if Rover.nav_angles is not None:
        # Check for Rover.mode status
        if Rover.mode == 'forward': 
            move_forward(Rover)

        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop':
            stop_moving(Rover)
            
        elif Rover.mode == 'reverse':
            move_reverse(Rover)
            
    # Just to make the rover do something 
    # even if no modifications have been made to the code
    else:
        Rover.throttle = 0 #Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0
        
    # If in a state where want to pickup a rock send pickup command
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True
    
    return Rover

