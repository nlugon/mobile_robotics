import numpy as np

def speed_conversion(speed):
    #speed=[left,right] in percentage
    #return [speed[0]/30, speed[1]/30]
    return [speed[0], speed[1]]


#might not work if it's a right turn (speedR < speedL)
def pos_estimate(position,angle,speed,period):
    #position = [x,y], last position
    #angle, last angle in radians
    #speed = [left,right], speed of motors since last position
    #period, time in seconds since last position

    D = 95 #distance between wheels in mm

    speed_mm = speed_conversion(speed)
    dist_l = speed_mm[0]*period
    dist_r = speed_mm[1]*period

    angle2 = angle + (dist_r-dist_l)/D

    R_turn = D/2
    if dist_r == dist_l:
        x2 = position[0] - dist_l*np.sin(angle)
        y2 = position[1] + dist_l*np.cos(angle)
    else:
        R_turn = D/2 + dist_l/((dist_r-dist_l)/D) #radius of the turn
        x2 = position[0] + R_turn*(np.cos(angle2) - np.cos(angle))
        y2 = position[1] + R_turn*(np.sin(angle2) - np.sin(angle))
    print("Turn Radius: " + str(R_turn))

    return [x2,y2],angle2

if __name__ == "__main__":
    D = 95
    position,angle = pos_estimate([0,0],0,[D*np.pi/8,-D*np.pi/8],1)
    print("Position: {0}\nAngle: {1} degrees".format(position,angle*180/np.pi))
    position,angle = pos_estimate(position,angle,[50,50],1)
    print("Position: {0}\nAngle: {1} degrees".format(position,angle*180/np.pi))