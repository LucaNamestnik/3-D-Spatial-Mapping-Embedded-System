Embedded Spatial Measurement System - 2DX3 Microprocessor Systems Project
Overview
This project is an Embedded Spatial Measurement System designed to capture 3D spatial data in indoor environments using LIDAR technology. The system utilizes a Time of Flight (ToF) sensor mounted on a stepper motor to acquire distance measurements across multiple planes. These measurements are transmitted via UART to a Python script, where the data is processed and visualized as a 3D point cloud using Open3D. By rotating the ToF sensor across the y-z plane in 11.25° increments, the system collects 32 distance measurements per plane. Along the x-axis, measurements are taken by manually repositioning the device, and the complete dataset is used to generate a detailed 3D model of the environment.

Features
The system offers 360-degree spatial measurement capabilities using a stepper motor and ToF sensor. It utilizes LIDAR-based distance measurement with the VL53L1X ToF sensor, which has a maximum range of 4000mm. Data is transmitted between the microcontroller (MSP432E401Y) and a Python script via UART communication for data collection and processing. The 3D visualization of the scanned environment is performed using Python's Open3D library, creating an accurate point cloud representation. Additionally, the system includes real-time status updates through onboard LEDs, providing feedback during the measurement process.

Technical Skills
This project requires a range of technical skills. Embedded systems programming is used to control the Texas Instruments MSP432E401Y microcontroller, with coding performed in Keil uVision using C. The system employs I2C communication to interface with the VL53L1X ToF sensor for distance measurement, while UART communication is used for data transmission from the microcontroller to the PC. The stepper motor control involves precise rotation to capture spatial data, and Python scripting is utilized for data processing and visualization. The Python script communicates with the microcontroller through Pyserial, processes the incoming data, and generates 3D models using Open3D.

Components Used
The key hardware components include the MSP432E401Y microcontroller, which controls the system, and the VL53L1X ToF sensor, responsible for measuring distances up to 4000mm. The 28BYJ-48 stepper motor controls the sensor's rotation, and several 10KΩ pull-up resistors are used to facilitate I2C communication. On the software side, the project requires Python 3.6+, along with the Pyserial and Open3D libraries for data collection and 3D visualization.

How It Works
The system begins by scanning a vertical plane using the ToF sensor, rotating in 11.25° increments for a total of 32 measurements per plane. After completing each scan, the system pauses and waits for the user to reposition the device along the x-axis, after which the measurements resume. Once all planes have been measured, the Python script processes the data into 3D coordinates, which are then visualized using Open3D. The final output is a detailed 3D model of the scanned environment.

Usage
To use the system, the hardware must be set up according to the provided schematic. After installing Python 3.6+ and the required dependencies (pyserial and open3d), users can run the provided Python script to control the microcontroller. The system begins scanning when the Enter key is pressed. Throughout the scanning process, real-time status feedback is provided via onboard LEDs, and users can adjust settings such as scan resolution and the number of planes to be measured. Once scanning is complete, the 3D model is automatically generated and displayed on the screen.

Applications
This system has a wide range of real-world applications. It can be used for room mapping, creating accurate 3D models of indoor spaces, or for measuring dimensions in loading docks to ensure vehicles fit correctly. Additionally, it can be applied to create 3D models of various objects for reverse engineering, or for precise measurement in prosthetic design, ensuring proper attachment.

Limitations
One limitation of the system is the maximum range of the ToF sensor, which is restricted to 4000mm. Additionally, in brightly lit environments, the sensor’s performance may be reduced. The microcontroller’s floating-point precision can introduce small errors in measurement accuracy, especially when performing complex trigonometric calculations. The system’s speed is also limited by the ToF sensor’s maximum measurement frequency of 50Hz, and by the rotation speed of the stepper motor, which adds delays between each measurement.
