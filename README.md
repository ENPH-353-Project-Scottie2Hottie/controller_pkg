<div id="top"></div>
<!--
*** Thanks for checking out the Best-README-Template. If you have a suggestion
*** that would make this better, please fork the repo and create a pull request
*** or simply open an issue with the tag "enhancement".
*** Don't forget to give the project a star!
*** Thanks again! Now go create something AMAZING! :D
-->



<!-- PROJECT SHIELDS -->
<!--
*** I'm using markdown "reference style" links for readability.
*** Reference links are enclosed in brackets [ ] instead of parentheses ( ).
*** See the bottom of this document for the declaration of the reference variables
*** for contributors-url, forks-url, etc. This is an optional, concise syntax you may use.
*** https://www.markdownguide.org/basic-syntax/#reference-style-links
-->


<!-- PROJECT LOGO -->
<br />
<div align="center">
  <a href="https://github.com/ENPH-353-Project-Scottie2Hottie/controller_pkg">
    <img src="logo.png" alt="Logo" width="80" height="80">
  </a>

<h3 align="center">Autonomous Parking Robot</h3>

  <p align="center">
    Autonomous controller for a parking robot tasked with reading license plates within a ROS – simulated parking lot using 2D image processing and machine learning with Python 
    <br />
    <a href="https://github.com/ENPH-353-Project-Scottie2Hottie/controller_pkg"><strong>Explore the docs »</strong></a>
    <br />
    <br />
  </p>
</div>



<!-- TABLE OF CONTENTS -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#about-the-project">About The Project</a>
      <ul>
        <li><a href="#built-with">Built With</a></li>
      </ul>
    </li>
    <li>
      <a href="#getting-started">Getting Started</a>
      <ul>
        <li><a href="#prerequisites">Prerequisites</a></li>
        <li><a href="#installation">Installation</a></li>
      </ul>
    </li>
    <li><a href="#usage">Usage</a></li>
    <li><a href="#contact">Contact</a></li>
  </ol>
</details>

<!-- ABOUT THE PROJECT -->
## About The Project

![Simulation Overhead View](https://github.com/ENPH-353-Project-Scottie2Hottie/controller_pkg/blob/main/sim_overhead_view.png?raw=true)

This controller package contains the ROS nodes used to autonomously control the parking robot. Our parking robot is capable of autonomously navigating the simulation arena and simultaneously read and report the location and license plate of the cars scattered around the arena.

NOTE: Must use Lubuntu 18.04

* [Competition Rules](https://github.com/ENPH-353-Project-Scottie2Hottie/controller_pkg/blob/main/Competition_Rules.pdf)
* [Final Report](https://github.com/ENPH-353-Project-Scottie2Hottie/controller_pkg/blob/main/Final_Report.pdf)

<p align="right">(<a href="#top">back to top</a>)</p>

### Built With

* [Python 2.7](https://www.python.org/download/releases/2.7/)
* [ROS Melodic](http://wiki.ros.org/melodic)
* [TensorFlow](https://www.tensorflow.org/)
* [OpenCV](https://opencv.org/)


<p align="right">(<a href="#top">back to top</a>)</p>

<!-- GETTING STARTED -->
## Getting Started

Refer to [Competition Rules](https://github.com/ENPH-353-Project-Scottie2Hottie/controller_pkg/blob/main/Competition_Rules.pdf) for instructions on how to set up ROS environment

### Prerequisites

* Lubuntu 18.04
* Python 2.7
* ROS Melodic
* TensorFlow
* OpenCV

### Installation

1. Ensure that the ROS environment is set up
2. Change directory to src folder in ROS workspace
   ```sh
   cd ~/ros_ws/src
   ```

3. Create new package for controller
   ```sh
   catkin_create_pkg controller_pkg rospy
   ```
   ```sh
   cd ~/ros_ws
   ```
   ```sh
   catkin_make
   ```

4. Clone the repo to overwrite new package
   ```sh
   cd ~/ros_ws/src
   ```
   ```sh
   git clone https://github.com/ENPH-353-Project-Scottie2Hottie/controller_pkg.git
   ```
5. Build the environment again
   ```sh
   cd ~/ros_ws
   ```
   ```sh
   catkin_make
   ```

<p align="right">(<a href="#top">back to top</a>)</p>

<!-- USAGE EXAMPLES -->
## Usage

NOTE: In every new terminal tab/window, source the environment:
   ```sh
   source ~/ros_ws/devel/setup.bash
   ```

1. Start simulated world
   ```sh
   ~/ros_ws/src/2020_competition/enph353/enph353_utils/scripts/run_sim.sh -vpg
   ```
   
2. Start the score tracking app (Open a new tab in the current terminal window with Ctrl+Shift+T)
   ```sh
   ~/ros_ws/src/2020_competition/enph353/enph353_utils/scripts/score_tracker.py
   ```
   
3. Start robot controller
   ```sh
   roslaunch controller_pkg pid.launch
   ```

<p align="right">(<a href="#top">back to top</a>)</p>