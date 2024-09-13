
## Title O3D Create Test Data
#
#   Purpose: This example Python program simulates data received from a
#   sensor by writing the expected format of example data to a file. The
#   visualization will be demonstrated using a Python module called Open3D.
#
#   Special notes:
#       1. Open3D only works with Pythons 3.6-3.9.  It does not work with 3.10
#       2. For this eample you should run it in IDLE.  Anaconda/Conda/Jupyter
#       require different Open3D graphing methods (these methods are poorly documented)
#       3. Under Windows 10 you may need to install the MS Visual C++ Redistributable bundle
#           https://docs.microsoft.com/en-us/cpp/windows/latest-supported-vc-redist?view=msvc-170
#       4. VirtualBox does not support OpenGL. If you're running Windows under Virtualbox or
#       your system doesn't support OpenGL (very rare), then you can install an OpenGL emulator dll
#           https://fdossena.com/?p=mesa/index.frag (unzip and copy opengl32.dll into Python dir)
#
#   T. Doyle
#   March 18, 2022 (Updated 2020 example)


import numpy as np
import open3d as o3d
import serial
import math

print_distance_only = True

ser = serial.Serial('COM7', baudrate=115200, timeout=10)
input("Press Enter to receive scanning configuration...")
ser.write('s'.encode())
ser.reset_output_buffer()
ser.reset_input_buffer()
line = ser.readline().decode()
num_planes = int(line.split(" ")[0])
x_step_size = int(line.split(" ")[1])
scans_per_plane = int(line.split(" ")[2])
print("The system will scan", num_planes, "planes with each plane being separated by", x_step_size, "mm and", scans_per_plane, "scans in each plane")

all_planes = []
ser.reset_output_buffer()
ser.reset_input_buffer()

for i in range(num_planes):
    print("Ready to scan plane", i+1)
    input("Press Enter to start scanning plane...")
    current_plane = []
    ser.write('s'.encode())
    for j in range(scans_per_plane):
        line = ser.readline().decode()
        index = line.split(", ")[0]
        distance = int(line.split(", ")[2])
        #if int(line.split(", ")[1]) != 0:
         #   distance = 4000
        if print_distance_only:
            print(index, distance)
        else:
            print(line)
        current_plane.append(distance)
    #if i % 2 == 1:
        #current_plane.reverse()
    all_planes.extend(current_plane)
    print("Completed scanning plane", i+1)

ser.close()

if __name__ == "__main__":

    data_filename = "scanData.xyz"
    
    f = open(data_filename, "w")

    for i in range(scans_per_plane*num_planes):
        x = math.floor(i / scans_per_plane) * x_step_size
        y = all_planes[i] * math.sin(2*math.pi*i/scans_per_plane)
        z = all_planes[i] * math.cos(2*math.pi*i/scans_per_plane)
        f.write('{} {} {}\n'.format(x, y, z))
    
    f.close()

    
    pcd = o3d.io.read_point_cloud(data_filename, format="xyz")

    yz_slice_vertex = []
    for i in range(0,scans_per_plane*num_planes):
        yz_slice_vertex.append([i])

    lines = []
    for i in range(0, scans_per_plane*num_planes, scans_per_plane):
        for j in range(scans_per_plane):
            if (j == scans_per_plane - 1):
                lines.append([yz_slice_vertex[i+j], yz_slice_vertex[i]])
            else:
                lines.append([yz_slice_vertex[i+j], yz_slice_vertex[i+j+1]])
    
    for i in range(scans_per_plane):
        for j in range(0, scans_per_plane*(num_planes-1), scans_per_plane):
            lines.append([yz_slice_vertex[i+j], yz_slice_vertex[i+j+scans_per_plane]])

    line_set = o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(np.asarray(pcd.points)),lines=o3d.utility.Vector2iVector(lines))
    o3d.visualization.draw_geometries([line_set])
