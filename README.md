# Robot Navigation Program
<a name="readme-top"></a>

## Table of Contents
* [Introduction](#introduction)
* [Technologies](#technologies)
* [Installation](#installation)
* [Usage](#usage)
* [Features](#features)
* [Credits](#credits)

## Introduction
This program is implemented on an eebot mobile robot to navigate through a maze and find the exit.

## Technologies
The project is created with:
* Assembly Language
* An eebot with a HCS12 Microcontroller
* EVALH1 trainer board
* CodeWarrior IDE

<p align="right">(<a href="#readme-top">back to top</a>)</p>

## Installation
Clone the repo:

`git clone https://github.com/icejan/Microprocessor-Systems/tree/main/Project/sources.git`

<p align="right">(<a href="#readme-top">back to top</a>)</p>

## Usage
To run the program, upload the source code into the eebot using CodeWarrior.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

## Features

https://github.com/icejan/Microprocessor-Systems/assets/97641242/1d3360df-7ed1-4a6d-bb2c-1294396c89dd

* The robot tracks the black guidance line and adjusts its position so that it stays parallel to the path
* When the robot encounters a new intersection, it prioritizes in turning left over turning right
* When the robot reaches a dead end, it turns around and remembers the wrong turn it previously made
* At the end, the robot finds its way to the exit of the maze

<p align="right">(<a href="#readme-top">back to top</a>)</p>

## Credits
* Dalton Crowe (Co-developer) - https://www.linkedin.com/in/daltoncrowe/
