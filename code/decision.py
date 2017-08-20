import numpy as np


# This is where you can build a decision tree for determining throttle, brake and steer
# commands based on the output of the perception_step() function
def decision_step(Rover):
    # Stop if we are close enough to pick up a sample
    if (Rover.near_sample):
        Rover.mode = 'stop'

    # Check if we have vision data to make decisions with
    if Rover.nav_angles is not None:
        # Check for Rover.mode status
        if Rover.mode == 'forward':
            if len(Rover.nav_angles) >= Rover.stop_forward: # Check the extent of navigable terrain
                # Set steering to average angle clipped to the range +/- 15
                Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                Rover.brake = 0

                # If mode is forward, navigable terrain looks good
                # and velocity is below max, then throttle
                if np.count_nonzero(Rover.nav_dists < 25) > 250:
                    Rover.throttle = -1
                    Rover.mode = 'spin'
                else:
                    if (Rover.max_vel < 0.2):
                        Rover.throttle = 0
                        Rover.brake = 10
                    else:
                        # Use proportional control for throttle setting
                        Rover.throttle = (Rover.max_vel - Rover.vel) / 1.25

            # If there's a lack of navigable terrain pixels then go to 'spin' mode
            elif len(Rover.nav_angles) < Rover.stop_forward:
                Rover.mode = 'spin'
        elif Rover.mode == 'spin':
            # spin mode: rotate until we have enough space in front of us
            if np.count_nonzero(Rover.nav_dists < 25) < 225:
                Rover.mode = 'forward'
            else:
                Rover.throttle = 0
                if (Rover.vel <= 0.2):
                    Rover.brake = 0
                    Rover.steer = -15
                else:
                    Rover.brake = 10
        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop':
            # If we're in stop mode but still moving keep braking
            if Rover.vel > 0.2 or Rover.near_sample:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:
                # Now we're stopped and we have vision data to see if there's a path forward
                if len(Rover.nav_angles) < Rover.go_forward:
                    Rover.mode = 'spin'
                else: # If we're stopped but see sufficient navigable terrain in front then go!
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                    Rover.mode = 'forward'
    else:
        # if we don't have vision data, don't move
        Rover.throttle = 0
        Rover.steer = 0
        Rover.brake = 0

    # If in a state where want to pickup a rock send pickup command
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True
        Rover.mode = 'forward'

    return Rover
