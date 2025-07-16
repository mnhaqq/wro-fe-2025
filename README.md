# Lonely At The Top
This is the official repository of team LONELY AT THE TOP for the World Robot Olympiad Future Engineers category. This repository contains everything related to the development of the robot. And this README contains the official documentation of the team.

# 📜 Repository Overview

Welcome to our robotics project repository! Here's a breakdown of the contents:

- **`schemes/`**  
  Contains the circuit diagrams used in our robot.

- **`models/`**  
  Includes all the 3D printed parts of the robot that will be taken to the international competition.

- **`src/`**  
  All source code for both challenges — with and without obstacles.

- **`t-photos/`**  
  Photos of the team, including one formal shot and one funny shot.

- **`v-photos/`**  
  Photos of the robot from every angle, including previous versions.

- **`video/`**  
  Contains videos where you can see our robot in action, completing both challenges.

- **`README.md`**  
  Our complete development journey! Here we explain every aspect of building the robot — from design to implementation.

# Components
A list of all the electrical components in the robot.
<table>
  <tr>
    <td><strong>Step Down Module</strong></td>
    <td><strong>Raspberry Pi 5</strong></td>
    <td><strong>Raspberry Pi 12MP Camera Module 3</strong></td>
  </tr>
  <tr>
    <td><img src="other/readme-images/step-down.png" width="200"/></td>
    <td><img src="other/readme-images/raspberry-pi-5-raspberry-pi-40958498898115.jpg" width="200"/></td>
    <td><img src="other/readme-images/raspberry-pi-camera-module-3-raspberry-pi-sc0874-43251879215299.webp" width="200"/></td>
  </tr>
  <tr>
    <td><strong>Arduino Nano</strong></td>
    <td><strong>10k ohm resistor</strong></td>
    <td><strong>Push Button</strong></td>
  </tr>
  <tr>
    <td><img src="other/readme-images/arduino-nano.jpg" width="200"/></td>
    <td><img src="other/readme-images/resistors.jpg" width="200"/></td>
    <td><img src="other/readme-images/push-button.jpeg" width="200"/></td>
  </tr>
  <tr>
    <td><strong>USB cable A to USB Mini B</strong></td>
    <td><strong>4x 18650 Batteries</strong></td>
    <td><strong>Jumper Wires</strong></td>
  </tr>
  <tr>
    <td><img src="other/readme-images/usb-cable.webp" width="200"/></td>
    <td><img src="other/readme-images/18650%203000mah%20batterys-1000x1000.png" width="200"/></td>
    <td><img src="other/readme-images/jumper-wires.webp" width="200"/></td>
  </tr>
  <tr>
    <td><strong>Raspberry Pi FPC Camera Cable</strong></td>
    <td><strong>LaTrax Rally 75054</strong></td>
    <td></td>
  </tr>
  <tr>
    <td><img src="other/readme-images/pi-camera-cable.jpg" width="200"/></td>
    <td><img src="other/readme-images/tra75054-5-grn_2.jpg" width="200"/></td>
    <td></td>
  </tr>
</table>

Below we show how we connect all these componenents
## 🧩 Use of Each Component

### 🔌 DC-DC XL4015 Adjustable Step-down Module (5A 75W with Digital Voltmeter)
This power module is used to **step down the voltage** from our 2s2p 18650 battery pack (8.4V when fully charged) to a stable **5.2V**, which is ideal for powering the **Raspberry Pi 5**.  
The built-in digital display helps us monitor the output voltage easily and adjust it precisely, ensuring the Raspberry Pi receives clean and safe power throughout operation.

---

### 🧠 Raspberry Pi 5
This serves as the **main processing unit or brain** of our robot. It handles:
- Real-time image processing from the camera module  
- Sensor data interpretation  
- Pathfinding algorithms and decision-making logic  
- Communication with the Arduino Nano  

We chose the Raspberry Pi 5 due to its powerful processor, increased RAM, and GPIO compatibility, making it perfect for compute-heavy robotics tasks.

---

### 📷 Raspberry Pi 12MP Camera Module 3
This camera captures **high-resolution images** of the game field, which are then processed by the Raspberry Pi for tasks such as:
- Line and object detection  
- Path planning  
- Visual-based decisions  

Its high quality and native Pi compatibility make it ideal for real-time computer vision applications.

---

### 🧰 Arduino Nano
The **Arduino Nano** is used as a dedicated microcontroller to:
- Interface with motor drivers  
- Control the steering servo and drive motors  
- Execute low-level, timing-critical control tasks  

This division of labor makes the control system more efficient and responsive.

---

### 📏 10kΩ Resistor
We use this as a **pull-down resistor** in our push button circuit to ensure that the GPIO pin connected to the button reads a stable LOW state when the button is not pressed, preventing **false signals**.

---

### 🔘 Push Button
A simple button used to **start or stop the robot's program**.  
It gives us manual control over the code execution without needing remote access or a keyboard.

---

### 🔋 18650 Lithium-ion Batteries (2s2p Configuration)
We wired four 18650 cells in a **2-series 2-parallel** configuration to:
- Get a nominal voltage of 7.4V (8.4V when fully charged)  
- Double the capacity and current output  

This ensures sufficient runtime and power delivery for the Raspberry Pi and motors.

---

### 🧵 Jumper Wires
Used to connect various components such as:
- Sensors to the Raspberry Pi  
- GPIO pins to the button and ultrasonic module  

They offer flexibility and are great for testing and prototyping.

---

### 🔌 USB Cable A to Mini B
This cable connects the **Raspberry Pi to the Arduino Nano**, enabling:
- Power supply to the Arduino  
- Serial communication for sending control commands  

Essential for integrating the two microcontrollers.

---

### 🚗 LaTrax Rally 75054 Chassis
This is a **commercial RC car chassis** we used as the robot's base. It provides:
- A strong and stable platform  
- Built-in drivetrain and suspension  
- Easy mounting for electronics and sensors  

It reduced the time spent on mechanical design and improved mobility and reliability.

---

### 🎥 Raspberry Pi FPC Camera Cable (Standard-Mini)
This flexible flat ribbon cable connects the **Camera Module 3** to the Raspberry Pi.  
The mini connector fits perfectly with our camera, and the cable length allows us to position the camera optimally for capturing the game field.

---

# Circuit Diagram
![](other/readme-images/circuit-diagram.jpg)

This circuit diagram represents the core electronic system of our robot, powered by two distinct battery packs and controlled using both an **Arduino Nano** and a **Raspberry Pi 5**. Each component is strategically connected to fulfill its specific function in the robot's operation.

### ⚙️ Power Supply and Regulation

- The system uses two battery packs:
  - A **2S2P 18650 Li-ion battery pack** to power the Raspberry Pi through a **DC-DC XL4015 step-down converter**, which regulates the voltage from 8.4V down to 5.2V to ensure safe operation of the Pi.
  - A **6000mAh 7.4V LiPo battery** is dedicated to powering the **DC motor** via an **ESC (Electronic Speed Controller)**, providing the high current required for reliable movement and acceleration.

### 🧠 Microcontroller & Microprocessor

- The **Raspberry Pi 5** serves as the central processing unit of the robot. It:
  - Processes data from the **Raspberry Pi Camera Module**
  - Sends movement instructions to the **Arduino Nano**
  - Interfaces with input components like the **push button** through GPIO pins

- The **Arduino Nano** is responsible for:
  - Receiving control commands from the Raspberry Pi
  - Driving the **servo motor** for steering
  - Sending PWM signals to the **ESC** to control the DC motor
  - Handling low-level real-time tasks for precise control

### 📷 Camera Integration

A **12MP Camera Module 3** is connected to the Raspberry Pi using a **FPC ribbon cable**. This camera captures visual data from the field for image processing tasks such as:
- Line tracking
- Object detection
- Lane following and obstacle avoidance

This enables the robot to make autonomous navigation decisions based on what it "sees."

### 🔁 Actuation

- A **DC motor**, connected through an **ESC**, provides torque and speed for forward and backward movement.
- A **servo motor** controls the steering mechanism of the chassis, allowing the robot to make turns with precision.

### 🎮 Input and Control

- A **push button** is connected to the Raspberry Pi via GPIO pins and a **10kΩ resistor** (as a pull-down resistor) to prevent floating values. This button is used to **start or stop** the program execution manually.
- **Jumper wires (UTP)** are used to make precise connections between components like the sensors, Arduino, and Raspberry Pi, ensuring signal integrity and clean communication.

---

### ✅ Summary

This modular design ensures:
- **Separation of logic and power domains**, preventing electrical noise or surges from affecting control logic
- **Efficient power delivery** to both the Raspberry Pi and motors
- **Clear division of responsibilities** between the Raspberry Pi (for high-level processing) and Arduino (for motor control)
- **Scalability**, allowing us to easily modify or extend the system for future challenges

The diagram illustrates a robust and well-integrated control system suitable for real-world robotic competitions, ensuring stability, performance, and flexibility in operation.


# 3D Parts
| Part       | Description              |
|------------|--------------------------|
| ![](other/readme-images/Camera-mold.png) | This 3D-printed component serves as a protective and structural housing for the camera, referred to as the camera mold. It is angled at approximately 170 degrees to optimize the camera's field of view, enabling better visibility of the game field during operation. |
| ![](other/readme-images/camera-support-frame.png) | This is the camera support frame, designed to pair with the camera mold to ensure stable and accurate camera positioning. Its adjustable height mechanism allows fine-tuning of the camera's elevation to achieve the desired field of view, enhancing visibility of the game field. |
| ![](other/readme-images/lower-pillars.png) | These pillars serve as the structural supports for the robot, holding up each deck and ensuring overall stability. Designed to bear the weight of the stacked components, the eight pillars also provide connection points that keep the entire robot assembly securely integrated |
| ![](other/readme-images/power-supply-platform.png) | This is the second deck (counting from the top), designed to hold the step-down power module and the batteries that supply power to the Raspberry Pi 5. It provides a stable platform for power distribution components and ensures efficient use of vertical space in the robot’s design. |
| ![](other/readme-images/top-platform.png) | This is the first deck (topmost level), which houses the Raspberry Pi 5, Arduino Nano, and a control button. It also interfaces with the camera support frame, featuring a sliding mechanism that allows the frame to be adjusted forward or backward as needed to optimize the camera’s positioning. |
| ![](other/readme-images/support-frame.png) | Serve as the foundation for the robot's structure. Provides support for all other printed components and form the critical connection between the RC chassis and the custom 3D-printed assemblies, ensuring a stable and integrated build.  |
| ![](other/readme-images/mount.png) | Combined with the support frame above to perform it's function. There's two of this part |

# Code And Programming
Our code is separated into two parts in two folders. The Pi Arduino Interface and the Main Code. Let's look at each separately

## PiArduinoInterface

