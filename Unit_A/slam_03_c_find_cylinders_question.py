# For each cylinder in the scan, find its ray and depth.
# 03_c_find_cylinders
# Claus Brenner, 09 NOV 2012
from pylab import *
from lego_robot import *

# Find the derivative in scan data, ignoring invalid measurements.
def compute_derivative(scan, min_dist):
    jumps = [ 0 ]
    for i in xrange(1, len(scan) - 1):
        l = scan[i-1]
        r = scan[i+1]
        if l > min_dist and r > min_dist:
            derivative = (r - l) / 2.0
            jumps.append(derivative)
        else:
            jumps.append(0)
    jumps.append(0)
    return jumps

# For each area between a left falling edge and a right rising edge,
# determine the average ray number and the average depth.
#Steps:
#1. Find the negative or positive edge
#2. When Negative or Positive Edge is found travel to 0.0 from there
#calculate the tuples till reching postive or negative edge. 
def find_cylinders(scan, scan_derivative, jump, min_dist):
    cylinder_list = []
    on_cylinder = False
    sum_ray, sum_depth, rays = 0.0, 0.0, 0
    min_threshold = 100.0
    for i in xrange(1, len(scan_derivative)):
        # --->>> Insert your cylinder code here.
        # Whenever you find a cylinder, add a tuple
        # (average_ray, average_depth) to the cylinder_list.

        # Just for fun, I'll output some cylinders.
        # Replace this by your code.
        if (scan_derivative[i]-scan_derivative[i-1] < -min_threshold and scan_derivative[i] < -min_threshold): #Negative Edge
            while (scan_derivative[i] < -min_threshold):
                i+=1
            flag = 0
            if (scan_derivative[i] > -min_threshold and i<len(scan_derivative)):
                while (scan_derivative[i] < min_threshold and scan_derivative[i] > -min_threshold):
                        print ('Neg', i, scan_derivative[i-1], scan_derivative[i], scan[i])
                        rays += 1
                        sum_ray += i
                        sum_depth += scan[i]
                        i+=1
                        if (i == len(scan_derivative)):
                            i -= 1
                            flag = 1
                            break    
                if (scan_derivative[i] > min_threshold and i<len(scan_derivative) or flag == 1): #Positive Edge detected
                    print ('Neg', (sum_ray/rays), (sum_depth/rays))
                    cylinder_list.append(((sum_ray/rays), (sum_depth/rays)))
                    sum_ray, sum_depth, rays = 0.0, 0.0, 0
                    i -= 1
    return cylinder_list


if __name__ == '__main__':

    minimum_valid_distance = 20.0
    depth_jump = 100.0

    # Read the logfile which contains all scans.
    logfile = LegoLogfile()
    logfile.read("robot4_scan.txt")

    # Pick one scan.
    scan = logfile.scan_data[73]
    print (scan)
    # Find cylinders.
    der = compute_derivative(scan, minimum_valid_distance)
    print (der)
    cylinders = find_cylinders(scan, der, depth_jump,
                               minimum_valid_distance)

    # Plot results.
    plot(scan)
    scatter([c[0] for c in cylinders], [c[1] for c in cylinders],
        c='r', s=200)
    show()