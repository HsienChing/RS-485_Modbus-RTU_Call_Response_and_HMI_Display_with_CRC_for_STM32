<!--
*** Thanks for checking out the Best-README-Template. If you have a suggestion
*** that would make this better, please fork the repo and create a pull request
*** or simply open an issue with the tag "enhancement".
*** Thanks again! Now go create something AMAZING! :D
Ref: https://github.com/othneildrew/Best-README-Template/edit/master/README.md
-->



<!-- PROJECT SHIELDS -->
<!--
*** I'm using markdown "reference style" links for readability.
*** Reference links are enclosed in brackets [ ] instead of parentheses ( ).
*** See the bottom of this document for the declaration of the reference variables
*** for contributors-url, forks-url, etc. This is an optional, concise syntax you may use.
*** https://www.markdownguide.org/basic-syntax/#reference-style-links
-->
<!-- 
[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![MIT License][license-shield]][license-url]
[![LinkedIn][linkedin-shield]][linkedin-url]
-->


<!-- PROJECT LOGO -->
<!--
<br />
<p align="center">
  <a href="https://github.com/othneildrew/Best-README-Template">
    <img src="images/logo.png" alt="Logo" width="80" height="80">
  </a>

  <h3 align="center">Best-README-Template</h3>

  <p align="center">
    An awesome README template to jumpstart your projects!
    <br />
    <a href="https://github.com/othneildrew/Best-README-Template"><strong>Explore the docs »</strong></a>
    <br />
    <br />
    <a href="https://github.com/othneildrew/Best-README-Template">View Demo</a>
    ·
    <a href="https://github.com/othneildrew/Best-README-Template/issues">Report Bug</a>
    ·
    <a href="https://github.com/othneildrew/Best-README-Template/issues">Request Feature</a>
  </p>
</p>
-->


<!-- TABLE OF CONTENTS -->
<!--
<details open="open">
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
    <li><a href="#roadmap">Roadmap</a></li>
    <li><a href="#contributing">Contributing</a></li>
    <li><a href="#license">License</a></li>
    <li><a href="#contact">Contact</a></li>
    <li><a href="#acknowledgements">Acknowledgements</a></li>
  </ol>
</details>
-->



<!-- ABOUT THE PROJECT -->
# RS-485 Modbus-RTU Call Response and HMI Display with CRC for STM32

# About the project

The STM32 NUCLEO-F446RE board is used to communicate with the external device through RS-485 Modbus-RTU. 
The device responses are resolved by the STM32 NUCLEO-F446RE with cyclic redundancy check (CRC) and outputted to the human-machine interface (HMI).


[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![Contributor Covenant](https://img.shields.io/badge/Contributor%20Covenant-2.0-4baaaa.svg)](code_of_conduct.md)


# Brief description

There are two boards used in this project, i.e., the main and test boards. 
The main board is the major developing object in this project, providing all required functionalities.
The test board is used to simulate the device for testing the functionalities of the main board.

## Main board

Suggested board: 
1. STM32 NUCLEO-F446RE
1. Blue Pill (STM32F103C8T6)

Suggested development environment:
1. STM32CubeIDE

Purpose: 
1. Send Modbus-RTU command to the device through RS-485 via the UART1.
1. Get responses from the device through RS-485.
1. Send the device responses to the human-machine interface (HMI) via the UART3.
1. Users can monitor the device responses from the STM32CubeIDE console monitor (or other serial monitor) via UART2.

Suggested hardware setup: 
1. UART1: A "RS-485 to TTL module" is used to convert the RS-485 signal because STM32 NUCLEO-F446RE (and Blue Pill) does not support RS-485 directly.
1. UART3: A HMI with the UART interface is used to show the device responses through the UART3.
    
Suggested software (STM32CubeIDE) setup:
1. Pinout & configuration/Connectivity
   1. Turn on USART1, Mode: Asynchronous, Basic parameters:   9600 8N1
   1. Turn on USART2, Mode: Asynchronous, Basic parameters: 115200 8N1
   1. Turn on USART3, Mode: Asynchronous, Basic parameters: 115200 8N1

Suggested library for calculating CRC:
1. The CRC 16 calculation function is available from Lammert Bies, https://github.com/lammertb/libcrc . 
1. On-line CRC calculation and free library, https://www.lammertbies.nl/comm/info/crc-calculation .
1. The key files of the library used in this project is put in the [library](library).

Notice:

If the following problems happen during compiling by STM32CubeIDE, go to "Project > Properties > C/C++ Build > Settings > Tool Settings > MCU Settings"
and then check the box "Use float with printf from newlib-nano (-u _printf_float)."

    1. The float formatting support is not enabled, check your MCU Settings from "Project Properties > C/C++ Build > Settings > Tool Settings", or add manually "-u _printf_float" in linker flags.	main.c	/F103C8_UART_HMI/Core/Src	
    2. Problem description: The float formatting support is not enabled, check your MCU Settings from "Project Properties > C/C++ Build > Settings > Tool Settings", or add manually "-u _printf_float" in linker flags.




## Test board (device simulator)

Suggested board: 
1. STM32 NUCLEO-F446RE
1. Blue Pill (STM32F103C8T6)

Suggested development environment:
1. STM32CubeIDE

Purpose: 
1. Receive Modbus-RTU command from the main board through RS-485 via the UART1.
1. Generate the Modbus data with CRC and counting value.
1. Send the Modbus data back.

Suggested hardware setup: 
1. UART1: A "RS-485 to TTL module" is used to convert the RS-485 signal because STM32 NUCLEO-F446RE (and Blue Pill) does not support RS-485 directly.
    
Suggested software (STM32CubeIDE) setup:
1. Pinout & configuration/Connectivity
   1. Turn on USART1, Mode: Asynchronous, Basic parameters:   9600 8N1

Suggested library for calculating CRC:
1. The CRC 16 calculation function is available from Lammert Bies, https://github.com/lammertb/libcrc . 
1. On-line CRC calculation and free library, https://www.lammertbies.nl/comm/info/crc-calculation .
1. The key files of the library used in this project is put in the [library](library).





<!-- GETTING STARTED -->
<!--
# Getting Started

This is an example of how you may give instructions on setting up your project locally.
To get a local copy up and running follow these simple example steps.
-->


<!-- USAGE EXAMPLES -->
# Usage
1. Setup the "Pinout & configuration" of the boards by STM32CubeIDE.
1. Copy and paste the source code into the file ProjectFolder/Core/Src/main.c .

The source codes are in the [code](code) directory. 
Example: "Main board - STM32 NUCLEO-F446RE" means that the STM32 NUCLEO-F446RE is used as the main board.

## Main board - Blue Pill (STM32F103C8T6)

| #    | Source file                                                                                        | Date       |
| ---- | -------------------------------------------------------------------------------------------------- | ---------- |
|    1 | [Main_Modbus_HMI_BluePill_v01.c](code/Main_board-Blue_Pill/Main_Modbus_HMI_BluePill_v01.c)         | 2021-09-15 |


## Main board - STM32 NUCLEO-F446RE

| #    | Source file                                                                                        | Date       |
| ---- | -------------------------------------------------------------------------------------------------- | ---------- |
|    1 | [Main_Modbus_HMI_F446RE_v01.c](code/Main_board-STM32_NUCLEO-F446RE/Main_Modbus_HMI_F446RE_v01.c)   | 2021-09-14 |


## Test board - Blue Pill (STM32F103C8T6)

| #    | Source file                                                                                        | Date       |
| ---- | -------------------------------------------------------------------------------------------------- | ---------- |
|    1 | [Test_Modbus_HMI_BluePill_v01.c](code/Test_board-Blue_Pill/Test_Modbus_HMI_BluePill_v01.c)         | 2021-09-14 |


## Test board - STM32 NUCLEO-F446RE

| #    | Source file                                                                                        | Date       |
| ---- | -------------------------------------------------------------------------------------------------- | ---------- |
|    1 | [Test_Modbus_HMI_F446RE_v01.c](code/Test_board-STM32_NUCLEO-F446RE/Test_Modbus_HMI_F446RE_v01.c)   | 2021-09-15 |


<!-- LICENSE -->
# License

Distributed under the [MIT License](LICENSE).



<!-- CONTACT -->
# Contact

Author: Dr. Hsien-Ching Chung

ORCID: https://orcid.org/0000-0001-9364-8858

Project Link: [https://github.com/HsienChing/RS-485_Modbus-RTU_Call_Response_and_HMI_Display_with_CRC_for_STM32](https://github.com/HsienChing/RS-485_Modbus-RTU_Call_Response_and_HMI_Display_with_CRC_for_STM32)



<!-- ACKNOWLEDGEMENTS -->
# Acknowledgements
<!--
* [GitHub Emoji Cheat Sheet](https://www.webpagefx.com/tools/emoji-cheat-sheet)
* [Img Shields](https://shields.io)
* [Choose an Open Source License](https://choosealicense.com)
* [GitHub Pages](https://pages.github.com)
* [Animate.css](https://daneden.github.io/animate.css)
* [Loaders.css](https://connoratherton.com/loaders)
* [Slick Carousel](https://kenwheeler.github.io/slick)
* [Smooth Scroll](https://github.com/cferdinandi/smooth-scroll)
* [Sticky Kit](http://leafo.net/sticky-kit)
* [JVectorMap](http://jvectormap.com)
* [Font Awesome](https://fontawesome.com)
-->
H.C. Chung thanks all the contributors to this project for their valuable discussions and recommendations, especially Jung-Feng Lin, Hsiao-Wen Yang, Yen-Kai Lo, An-De Andrew Chung.

This work was supported in part by Super Double Power Technology Co., Ltd., Taiwan under grant SDP-RD-PROJ-001-2020.



<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
<!--
[contributors-shield]: https://img.shields.io/github/contributors/othneildrew/Best-README-Template.svg?style=for-the-badge
[contributors-url]: https://github.com/othneildrew/Best-README-Template/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/othneildrew/Best-README-Template.svg?style=for-the-badge
[forks-url]: https://github.com/othneildrew/Best-README-Template/network/members
[stars-shield]: https://img.shields.io/github/stars/othneildrew/Best-README-Template.svg?style=for-the-badge
[stars-url]: https://github.com/othneildrew/Best-README-Template/stargazers
[issues-shield]: https://img.shields.io/github/issues/othneildrew/Best-README-Template.svg?style=for-the-badge
[issues-url]: https://github.com/othneildrew/Best-README-Template/issues
[license-shield]: https://img.shields.io/github/license/othneildrew/Best-README-Template.svg?style=for-the-badge
[license-url]: https://github.com/othneildrew/Best-README-Template/blob/master/LICENSE.txt
[linkedin-shield]: https://img.shields.io/badge/-LinkedIn-black.svg?style=for-the-badge&logo=linkedin&colorB=555
[linkedin-url]: https://linkedin.com/in/othneildrew
[product-screenshot]: images/screenshot.png
-->
