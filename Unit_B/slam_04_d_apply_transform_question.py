# For each cylinder in the scan, find its cartesian coordinates,
# in the world coordinate system.
# Find the closest pairs of cylinders from the scanner and cylinders
# from the reference, and the optimal transformation which aligns them.
# Then, use this transform to correct the pose.
# 04_d_apply_transform
# Claus Brenner, 14 NOV 2012
from lego_robot import *
from slam_b_library import filter_step
from slam_04_a_project_landmarks import\
     compute_scanner_cylinders, write_cylinders
from math import sqrt, atan2

# Given a list of cylinders (points) and reference_cylinders:
# For every cylinder, find the closest reference_cylinder and add
# the index pair (i, j), where i is the index of the cylinder, and
# j is the index of the reference_cylinder, to the result list.
# This is the function developed in slam_04_b_find_cylinder_pairs.
def find_cylinder_pairs(cylinders, reference_cylinders, max_radius):
    cylinder_pairs = []

    # --->>> Insert here your code from the last question,
    # slam_04_b_find_cylinder_pairs.
    for i in range(len(cylinders)):
        x, y = cylinders[i]
        arr = []
        for j in range(len(reference_cylinders)):
            x1, y1 = reference_cylinders[j]
            arr.append(distance_between(cylinders[i], reference_cylinders[j]))

        min_index = find_min(arr, max_radius)
        cylinder_pairs.append((i, min_index))    
    return cylinder_pairs

def distance_between(c, r):
    x1, y1 = c
    x2, y2 = r
    return (sqrt((x2-x1) ** 2 + (y2-y1) ** 2))

def find_min(arr, max_radius):
    min_n = 1000
    min_index = 0
    for n in range(len(arr)):
        if (arr[n] < max_radius and arr[n] <= min_n):
            min_n = arr[n]
            min_index = n
    return min_index

# Given a point list, return the center of mass.
def compute_center(point_list):
    # Safeguard against empty list.
    if not point_list:
        return (0.0, 0.0)
    # If not empty, sum up and divide.
    sx = sum([p[0] for p in point_list])
    sy = sum([p[1] for p in point_list])
    return (sx / len(point_list), sy / len(point_list))

# Given a left_list of points and a right_list of points, compute
# the parameters of a similarity transform: scale, rotation, translation.
# If fix_scale is True, use the fixed scale of 1.0.
# The returned value is a tuple of:
# (scale, cos(angle), sin(angle), x_translation, y_translation)
# i.e., the rotation angle is not given in radians, but rather in terms
# of the cosine and sine.
def estimate_transform(left_list, right_list, fix_scale = False):
    # Compute left and right center.
    print "Left list", left_list
    print "Right list", right_list
    lcx, lcy = compute_center(left_list)
    rcx, rcy = compute_center(right_list)
    # print ("left COM", compute_center(left_list))
    # print ("right COM", compute_center(right_list))
    lx_dash, rx_dash, ly_dash, ry_dash = [], [], [], []
    for l in left_list:
        lx_dash.append(l[0] - lcx)
        ly_dash.append(l[1] - lcy)
    for r in right_list:
        rx_dash.append(r[0] - rcx)
        ry_dash.append(r[1] - rcy)
    # --->>> Insert here your code to compute lambda, c, s and tx, ty.
    # print lx_dash, ly_dash
    # print rx_dash, ry_dash

    cc, ss, ll, rr = 0.0, 0.0, 0.0, 0.0
    for i in range(len(left_list)):
        cc += rx_dash[i] * lx_dash[i] + ry_dash[i] * ly_dash[i]
        ss += -rx_dash[i] * ly_dash[i] + ry_dash[i] * lx_dash[i]
        ll += lx_dash[i] * lx_dash[i] + ly_dash[i] * ly_dash[i]
        rr += rx_dash[i] * rx_dash[i] + ry_dash[i] * ry_dash[i] 
    if ll == 0 or rr == 0:
        return None
    cc_ss_sqaure = sqrt((cc ** 2 + ss ** 2))
    la = sqrt(rr/ ll)
    c = cc / cc_ss_sqaure
    s = ss / cc_ss_sqaure
    tx = rcx - la * c * lcx + la * s * rcy
    ty = rcy - la * s * lcx - la * c * rcy 
    return la, c, s, tx, ty

# Given a similarity transformation:
# trafo = (scale, cos(angle), sin(angle), x_translation, y_translation)
# and a point p = (x, y), return the transformed point.
def apply_transform(trafo, p):
    la, c, s, tx, ty = trafo
    lac = la * c
    las = la * s
    x = lac * p[0] - las * p[1] + tx
    y = las * p[0] + lac * p[1] + ty
    return (x, y)

# Correct the pose = (x, y, heading) of the robot using the given
# similarity transform. Note this changes the position as well as
# the heading.
def correct_pose(pose, trafo):
    la, c, s, tx, ty = trafo
    # --->>> This is what you'll have to implement.
    x, y = apply_transform(trafo, pose)
    theta = atan2(s, c)
    print ('Original Pose', pose)
    
    print ('New Pose', (x, y, theta))
    return (x, y, theta)  # Replace this by the corrected pose.


if __name__ == '__main__':
    # The constants we used for the filter_step.
    scanner_displacement = 30.0
    ticks_to_mm = 0.349
    robot_width = 150.0

    # The constants we used for the cylinder detection in our scan.    
    minimum_valid_distance = 20.0
    depth_jump = 100.0
    cylinder_offset = 90.0

    # The maximum distance allowed for cylinder assignment.
    max_cylinder_distance = 400.0

    # The start pose we obtained miraculously.
    pose = (1850.0, 1897.0, 3.717551306747922)

    # Read the logfile which contains all scans.
    logfile = LegoLogfile()
    logfile.read("robot4_motors.txt")
    logfile.read("robot4_scan.txt")

    # Also read the reference cylinders (this is our map).
    logfile.read("robot_arena_landmarks.txt")
    reference_cylinders = [l[1:3] for l in logfile.landmarks]

    out_file = file("apply_transform.txt", "w")
    for i in xrange(len(logfile.scan_data)):
        # Compute the new pose.
        pose = filter_step(pose, logfile.motor_ticks[i],
                           ticks_to_mm, robot_width,
                           scanner_displacement)

        # Extract cylinders, also convert them to world coordinates.
        cartesian_cylinders = compute_scanner_cylinders(
            logfile.scan_data[i],
            depth_jump, minimum_valid_distance, cylinder_offset)
        world_cylinders = [LegoLogfile.scanner_to_world(pose, c)
                           for c in cartesian_cylinders]

        # For every cylinder, find the closest reference cylinder.
        cylinder_pairs = find_cylinder_pairs(
            world_cylinders, reference_cylinders, max_cylinder_distance)

        # Estimate a transformation using the cylinder pairs.
        trafo = estimate_transform(
            [world_cylinders[pair[0]] for pair in cylinder_pairs],
            [reference_cylinders[pair[1]] for pair in cylinder_pairs],
            fix_scale = True)
        print (trafo)
        # Transform the cylinders using the estimated transform.
        transformed_world_cylinders = []
        if trafo:
            transformed_world_cylinders =\
                [apply_transform(trafo, c) for c in
                 [world_cylinders[pair[0]] for pair in cylinder_pairs]]

        # Also apply the trafo to correct the position and heading.
        if trafo:
            pose = correct_pose(pose, trafo)

        # Write to file.
        # The pose.
        print >> out_file, "F %f %f %f" % pose
        # The detected cylinders in the scanner's coordinate system.
        write_cylinders(out_file, "D C", cartesian_cylinders)
        # The detected cylinders, transformed using the estimated trafo.
        write_cylinders(out_file, "W C", transformed_world_cylinders)

    out_file.close()
