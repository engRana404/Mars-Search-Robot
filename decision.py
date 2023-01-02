import numpy as np

def decision_step(Rover):
   # Check if there are rocks
    if Rover.rock_angles is not None and len(Rover.rock_angles) > 0:
        Rover.mode = 'forward'
        Rover.steer = np.clip(np.mean(Rover.rock_angles * 180/np.pi), -15, 15)
        # Move towards the rock slowly
        if not Rover.near_sample:
            if Rover.vel < Rover.max_vel/2:
                Rover.brake = 0
                Rover.throttle = 0.1
            else:
                Rover.throttle = 0
                Rover.brake = 1
        # Stop when close to a rock.
        else:
            Rover.throttle = 0
            Rover.brake = Rover.brake_set

    
    elif Rover.nav_angles is not None:
        if Rover.mode == 'forward':
            if len(Rover.nav_angles) >= Rover.stop_forward:
                if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                    if Rover.throttle != 0 and Rover.vel < 0.01:
                        Rover.brake = 0
                        Rover.mode = 'stuck'
                    else:
                        # Set throttle value to throttle setting
                        Rover.throttle = Rover.throttle_set
                        Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -10, 10)                        

                else:  # Else coast
                    Rover.throttle = 0


                if len(Rover.nav_angles_left) > 2500:
                    Rover.brake = Rover.brake_set
                    Rover.mode = 'loop'

                if len(Rover.nav_angles_right) > len(Rover.nav_angles_left) > 1000:
                    Rover.throttle = 0
                    Rover.brake = Rover.brake_set
                    Rover.steer = 0
                    Rover.mode = 'change'
                    
                if (650 > Rover.nav_area > 10) :
                    Rover.throttle = 0
                    Rover.brake = 10
                    Rover.steer = 0
                    Rover.mode = 'stuck'  
                
                elif len(Rover.nav_angles) >= Rover.stop_forward:
				# If mode is forward, navigable terrain looks good
				# Except for start, if stopped means stuck.
				# Alternates between stuck and forward modes
                    if len(Rover.rock_angles) > 0:
                        drive_angles = Rover.rock_angles
                        drive_distance = np.min(Rover.rock_dists)
                    else:
                        drive_angles = Rover.nav_angles
                        drive_distance = Rover.dist_to_obstacle

				# Set throttle value to throttle setting
                    Rover.throttle = np.clip(drive_distance * 0.005 - Rover.vel * 0.2, 0, 2)
                    Rover.brake = 0
				# Set steering to average angle clipped to the range +/- 15
                    Rover.steer = np.clip(np.mean(drive_angles * 180 / np.pi), -15, 15)

                #Rover.brake = 0
                #Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -10, 10)

            elif len(Rover.nav_angles) < Rover.stop_forward:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
                Rover.mode = 'stop'


        elif Rover.mode == 'stuck' and len(Rover.rock_dists) == 0:
            if Rover.vel != 0:
                Rover.brake = 10
            Rover.steer = -15
            Rover.throttle = 0
            Rover.brake = 0
            if Rover.nav_area > 650:
                Rover.mode = 'forward'  

    

        elif Rover.mode == 'change':
            Rover.throttle = 0
            Rover.brake = Rover.brake_set
            Rover.steer = 0
            if len(Rover.nav_angles_left) >= len(Rover.nav_angles_left):
                Rover.throttle = 0
                Rover.brake = 0
                Rover.steer = -90
                if len(Rover.nav_angles_left) <= len(Rover.nav_angles_left):
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -10, 10)
                    Rover.mode = 'forward'
            else:
                Rover.throttle = 0
                Rover.brake = 0
                Rover.steer = 90
                Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -10, 10)
                Rover.mode = 'forward'


        elif Rover.mode == 'loop':
            if len(Rover.nav_angles_left) > len(Rover.nav_angles_left):
                Rover.brake = 0
                Rover.steer = -20
                Rover.mode = 'forward'
            else:
                Rover.brake = 0
                Rover.steer = 20
                Rover.mode = 'forward'

        elif Rover.mode == 'collect':
            if len(Rover.rock_angles) > 20:
                Rover.steer = -10
            elif len(Rover.rock_angles) < 20:
                Rover.steer = 10
        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop':
            # If we're in stop mode but still moving keep braking
            if Rover.vel > 0.2:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:
                # Now we're stopped and we have vision data to see if there's a path forward
                if len(Rover.nav_angles) < Rover.go_forward:
                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    Rover.steer = -30 # Could be more clever here about which way to turn
                # If we're stopped but see sufficient navigable terrain in front then go!
                if len(Rover.nav_angles) >= Rover.go_forward:
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -10, 10)
                    Rover.mode = 'forward'
    # Just to make the rover do something
    # even if no modifications have been made to the code
    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0

    # If in a state where want to pickup a rock send pickup command
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True

    return Rover
 