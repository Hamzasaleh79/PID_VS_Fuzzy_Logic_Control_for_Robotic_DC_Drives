# PID vs. Fuzzy Logic Control for Robotic DC Drives

This project demonstrates the implementation of PID and Fuzzy Logic controllers to maintain a DC motor at a target speed of 200 RPM. It includes adaptive gain scheduling for the PID controller and a rule-based approach for the Fuzzy Logic controller, both running on an Arduino with encoder feedback.

## Table of Contents

- [Introduction](#introduction)
- [Features](#features)
- [Installation and Setup](#installation-and-setup)
- [Usage](#usage)
- [Project Structure](#project-structure)
- [Contributing](#contributing)
- [License](#license)
- [Contact](#contact)
- [Acknowledgements](#acknowledgements)

## Introduction

This project explores two control strategies for regulating the speed of a DC motor: a traditional PID controller and a Fuzzy Logic controller. The objective is to maintain a steady speed of 200 RPM despite variations in load or disturbances.

The PID controller uses adaptive gain scheduling to dynamically adjust its parameters based on the error, ensuring optimal performance across different conditions. The Fuzzy Logic controller employs linguistic rules to manage the system's non-linearities. Both are implemented on an Arduino microcontroller with an encoder for precise speed measurement.

The project includes a LaTeX report detailing the algorithms, tuning methods, and performance comparisons, along with MATLAB plots visualizing the system's behavior.

## Features

- Implementation of PID and Fuzzy Logic controllers for DC motor speed control.
- Adaptive gain scheduling for the PID controller.
- Fuzzy rule-based control with an integral term for steady-state accuracy.
- Real-time speed measurement using an encoder.
- Comprehensive documentation with a LaTeX report and MATLAB plots.

## Installation and Setup

To set up the project, follow these steps:

1. **Clone the repository**:
   ```bash
   git clone https://github.com/Hamzasaleh79/PID_VS_Fuzzy_Logic_Control_for_Robotic_DC_Drives.git
