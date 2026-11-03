# PSOC&trade; Control C3 MCU: Trigger 

This example demonstrates how to use trigger multiplexer and one-to-one trigger with Infineon's PSOC&trade; Control C3 MCUs. This example configures three TCPWM counters and an ADC group to demonstrate the one-to-one and one-to-many trigger concept. The first TCPWM counter triggers two TCPWMs in one-to-many configuration and the same first TCPWM counter also triggers HPPASS SAR ADC Group 0 in one-to-one configuration.

[View this README on GitHub.](https://github.com/Infineon/mtb-example-ce238774-trigger)

[Provide feedback on this code example.](https://yourvoice.infineon.com/jfe/form/SV_1NTns53sK2yiljn?Q_EED=eyJVbmlxdWUgRG9jIElkIjoiQ0UyMzg3NzQiLCJTcGVjIE51bWJlciI6IjAwMi0zODc3NCIsIkRvYyBUaXRsZSI6IlBTT0MmdHJhZGU7IENvbnRyb2wgQzMgTUNVOiBUcmlnZ2VyIiwicmlkIjoiZGF2aWQuenVvQGluZmluZW9uLmNvbSIsIkRvYyB2ZXJzaW9uIjoiMi4wLjAiLCJEb2MgTGFuZ3VhZ2UiOiJFbmdsaXNoIiwiRG9jIERpdmlzaW9uIjoiTUNEIiwiRG9jIEJVIjoiSUNXIiwiRG9jIEZhbWlseSI6IlBTT0MifQ==)


## Requirements

- [ModusToolbox&trade;](https://www.infineon.com/modustoolbox) v3.8 or later (tested with v3.9)
- Board support package (BSP) minimum required version for :
   - KIT_PSC3M5_EVK: v3.3.0
   - KIT_PSC3M6_EVAL: v1.0.0
- Programming language: C
- Associated parts: PSOC&trade; Control C3 MCU Entry Line and PSOC&trade; Control C3 MCU Main Line parts


## Supported toolchains (make variable 'TOOLCHAIN')

- GNU Arm&reg; Embedded Compiler v14.2.1 (`GCC_ARM`) – Default value of `TOOLCHAIN`
- Arm&reg; Compiler v6.22 (`ARM`)
- IAR C/C++ Compiler v9.70.4 (`IAR`)


## Supported kits (make variable 'TARGET')

- [PSOC&trade; Control C3M5 Evaluation Kit](https://www.infineon.com/KIT_PSC3M5_EVK) (`KIT_PSC3M5_EVK`) – Default value of `TARGET`
- [PSOC&trade; Control C3M6 Evaluation Kit](https://www.infineon.com/KIT_PSC3M6_EVAL) (`KIT_PSC3M6_EVAL`)


## Hardware setup

This example uses the board's default configuration. See the kit user guide to ensure that the board is configured correctly.


## Software setup

See the [ModusToolbox&trade; tools package installation guide](https://www.infineon.com/ModusToolboxInstallguide) for information about installing and configuring the tools package.

Install a terminal emulator if you do not have one. Instructions in this document use [Tera Term](https://teratermproject.github.io/index-en.html).

This example requires no additional software or tools.


## Using the code example


### Create the project

The ModusToolbox&trade; tools package provides the Project Creator as both a GUI tool and a command line tool.

<details><summary><b>Use Project Creator GUI</b></summary>

1. Open the Project Creator GUI tool

   There are several ways to do this, including launching it from the dashboard or from inside the Eclipse IDE. For more details, see the [Project Creator user guide](https://www.infineon.com/ModusToolboxProjectCreator) (locally available at *{ModusToolbox&trade; install directory}/tools_{version}/project-creator/docs/project-creator.pdf*)

2. On the **Choose Board Support Package (BSP)** page, select a kit supported by this code example. See [Supported kits](#supported-kits-make-variable-target)

   > **Note:** To use this code example for a kit not listed here, you may need to update the source files. If the kit does not have the required resources, the application may not work

3. On the **Select Application** page:

   a. Select the **Application(s) Root Path** and the **Target IDE**

      > **Note:** Depending on how you open the Project Creator tool, these fields may be pre-selected for you

   b. Select this code example from the list by enabling its check box

      > **Note:** You can narrow the list of displayed examples by typing in the filter box

   c. (Optional) Change the suggested **New Application Name** and **New BSP Name**

   d. Click **Create** to complete the application creation process

</details>

<details><summary><b>Use Project Creator CLI</b></summary>

The 'project-creator-cli' tool can be used to create applications from a CLI terminal or from within batch files or shell scripts. This tool is available in the *{ModusToolbox&trade; install directory}/tools_{version}/project-creator/* directory.

Use a CLI terminal to invoke the 'project-creator-cli' tool. On Windows, use the command-line 'modus-shell' program provided in the ModusToolbox&trade; installation instead of a standard Windows command-line application. This shell provides access to all ModusToolbox&trade; tools. You can access it by typing "modus-shell" in the search box in the Windows menu. In Linux and macOS, you can use any terminal application.

The following example clones the "[PSOC&trade; Control C3 MCU: Trigger](https://github.com/Infineon/mtb-example-ce238774-trigger)" application with the desired name "PdlTrigger" configured for the *KIT_PSC3M5_EVK* BSP into the specified working directory, *C:/mtb_projects*:

   ```
   project-creator-cli --board-id KIT_PSC3M5_EVK --app-id mtb-example-ce238774-trigger --user-app-name PdlTrigger --target-dir "C:/mtb_projects"
   ```


The 'project-creator-cli' tool has the following arguments:

Argument | Description | Required/optional
---------|-------------|-----------
`--board-id` | Defined in the <id> field of the [BSP](https://github.com/Infineon?q=bsp-manifest&type=&language=&sort=) manifest | Required
`--app-id`   | Defined in the <id> field of the [CE](https://github.com/Infineon?q=ce-manifest&type=&language=&sort=) manifest | Required
`--target-dir`| Specify the directory in which the application is to be created if you prefer not to use the default current working directory | Optional
`--user-app-name`| Specify the name of the application if you prefer to have a name other than the example's default name | Optional

<br>

> **Note:** The project-creator-cli tool uses the `git clone` and `make getlibs` commands to fetch the repository and import the required libraries. For details, see the "Project creator tools" section of the [ModusToolbox&trade; tools package user guide](https://www.infineon.com/ModusToolboxUserGuide) (locally available at {ModusToolbox&trade; install directory}/docs_{version}/mtb_user_guide.pdf).

</details>


### Open the project

After the project has been created, you can open it in your preferred development environment.


<details><summary><b>Eclipse IDE</b></summary>

If you opened the Project Creator tool from the included Eclipse IDE, the project will open in Eclipse automatically.

For more details, see the [Eclipse IDE for ModusToolbox&trade; user guide](https://www.infineon.com/MTBEclipseIDEUserGuide) (locally available at *{ModusToolbox&trade; install directory}/docs_{version}/mt_ide_user_guide.pdf*).

</details>


<details><summary><b>Visual Studio (VS) Code</b></summary>

Launch VS Code manually, and then open the generated *{project-name}.code-workspace* file located in the project directory.

For more details, see the [Visual Studio Code for ModusToolbox&trade; user guide](https://www.infineon.com/MTBVSCodeUserGuide) (locally available at *{ModusToolbox&trade; install directory}/docs_{version}/mt_vscode_user_guide.pdf*).

</details>


<details><summary><b>Arm&reg; Keil&reg; µVision&reg;</b></summary>

Double-click the generated *{project-name}.cprj* file to launch the Keil&reg; µVision&reg; IDE.

For more details, see the [Arm&reg; Keil&reg; µVision&reg; for ModusToolbox&trade; user guide](https://www.infineon.com/MTBuVisionUserGuide) (locally available at *{ModusToolbox&trade; install directory}/docs_{version}/mt_uvision_user_guide.pdf*).

</details>


<details><summary><b>IAR Embedded Workbench</b></summary>

Open IAR Embedded Workbench manually, and create a new project. Then select the generated *{project-name}.ipcf* file located in the project directory.

For more details, see the [IAR Embedded Workbench for ModusToolbox&trade; user guide](https://www.infineon.com/MTBIARUserGuide) (locally available at *{ModusToolbox&trade; install directory}/docs_{version}/mt_iar_user_guide.pdf*).

</details>


<details><summary><b>Command line</b></summary>

If you prefer to use the CLI, open the appropriate terminal, and navigate to the project directory. On Windows, use the command-line 'modus-shell' program; on Linux and macOS, you can use any terminal application. From there, you can run various `make` commands.

For more details, see the [ModusToolbox&trade; tools package user guide](https://www.infineon.com/ModusToolboxUserGuide) (locally available at *{ModusToolbox&trade; install directory}/docs_{version}/mtb_user_guide.pdf*).

</details>


## Operation

1. Connect the board to your PC using the provided USB cable through the Debug USB connector on the board

2. Open a terminal program and select the Virtual COM port. Set the serial port parameters to 8N1 and 115200 baud

3. Program the board using one of the following:

   <details><summary><b>Using Eclipse IDE</b></summary>

      1. Select the application project in the Project Explorer

      2. In the **Quick Panel**, scroll down, and click **\<Application Name> Program (KitProg3_MiniProg4)** or **\<Application Name> Program (JLink)**
   </details>


   <details><summary><b>In other IDEs</b></summary>

   Follow the instructions in your preferred IDE

   </details>


   <details><summary><b>Using CLI</b></summary>

     From the terminal, execute the `make program` command to build and program the application using the default toolchain to the default target. The default toolchain is specified in the application's Makefile but you can override this value manually:
      ```
      make program TOOLCHAIN=<toolchain>
      ```

      Example:
      ```
      make program TOOLCHAIN=GCC_ARM
      ```
   </details>

4. After programming, the application starts automatically

5. Press the user switch (SW2). Confirm that the following is displayed on the UART terminal and the kit LED blinks

   **Figure 1. Terminal output**

   ![](images/terminal-output.png)


## Debugging

You can debug the example to step through the code.


<details><summary><b>In Eclipse IDE</b></summary>

Use the **\<Application Name> Debug (KitProg3_MiniProg4)** or **\<Application Name> Debug (JLink)** configuration in the **Quick Panel**. For details, see the "Program and debug" section in the [Eclipse IDE for ModusToolbox&trade; user guide](https://www.infineon.com/MTBEclipseIDEUserGuide).

</details>


<details><summary><b>In other IDEs</b></summary>

Follow the instructions in your preferred IDE.
</details>


## Design and implementation


### Trigger connections

- Use **TCPWM0 Group1 Counter 5** to generate out trigger signal

- Connect the out trigger to **TCPWM0 Group1 Counter 6** and **TCPWM0 Group2 Counter 0** via the **Trigger multiplexer group 10**, and connect the out trigger to **HPPASS SAR ADC Group0** via the **Trigger one-to-one group 2** as shown in **Figure 2**

**Figure 2. Trigger connection**

![](images/triggers_connection.svg)


### Configuration of peripherals

Configure the PWM1 user as shown in **Table 1** and **Figure 3**.

**Table 1. User PWM1 configuration**

Resource   | Name/value
:--------  |  :--------------
TCPWM[0] Group[1] 16-bit Counter 5 | USER_PWM1
Run Mode | One Shot
Interrupt Source | Overflow & Underflow
Trigger 1 Event | Overflow
Trigger 1 Signal  | Trigger 2 tr_b_in [USED] <br> TCPWM[0] Group[1] 16-bit Counter 6 <br> TCPWM[0] Group[2] 16-bit Counter 0

**Figure 3. User PWM 1 configuration**

![](images/pwm1_configuration.svg)


Configure the PWM2 user as shown in **Table 2** and **Figure 4**.

**Table 2. User PWM2 configuration**

Resource   | Name/value
:--------  |  :--------------
TCPWM[0] Group[1] 16-bit Counter 6 | USER_PWM2
Run Mode | Continuous
Start signal | TCPWM[0] Group[1] 16-bit Counter 5 out 1 (USER_PWM1) [SHARED]

**Figure 4. User PWM 2 configuration**

![](images/pwm2_configuration.svg)



Configure the timer as shown in **Table 3** and **Figure 5**.

**Table 3. User timer configuration**

Resource   | Name/value
:--------  |  :--------------
TCPWM[0] Group[2] 16-bit Counter 0 | USER_TIMER
Interrupt Source | Overflow & Underflow
Start signal | TCPWM[0] Group[1] 16-bit Counter 5 out 1 (USER_PWM1) [SHARED]

**Figure 5. User timer configuration**

![](images/timer_configuration.svg)



### Analog configuration

Configure the SAR ADC as shown in **Table 4** and **Figure 6**.

**Table 4. HPPASS configuration**

Resource   | Name/value
:--------  |  :--------------
Trigger 2 | TCPWM[0] Group[1] 16-bit Counter 5 out 1 (USER_PWM1) [SHARED]
Input trigger of Group 0 | Trigger 2

**Figure 6. HPPASS configuration**

![](images/hppass_configuration.svg)


**Figure 7** shows the flowchart of trigger application.

**Figure 7.  Flow diagram for the application**

![](images/flowchart.svg)


### Resources and settings

**Table 5. Application resources**

 Resource  |  Alias/object     |    Purpose
 :-------- | :-------------    | :------------
 UART (HAL) |DEBUG_UART_hal_obj   | UART HAL object used by Retarget-IO for the Debug UART port
 GPIO    | CYBSP_USER_LED              | User LED
 TCPWM0 Group1 Counter 5 | USER_PWM1   | Generate TCPWM out trigger to trigger other peripherals
 TCPWM0 Group1 Counter 6 | USER_PWM2   | TCPWM out trigger to trigger PWM starting by trigger multiplexer
 TCPWM0 Group2 Counter 0 | USER_TIMER  | TCPWM out trigger to trigger timer starting by trigger multiplexer
 HPPASS  | SAR ADC Group 0             | TCPWM out trigger to trigger SAR ADC group conversion by one-to-one trigger

<br>


## Related resources

Resources  | Links
-----------|----------------------------------
Documentation | [PSOC&trade; Control C3 MCU documents](https://documentation.infineon.com/psoccontrolc3/docs/kfc1732622054982)
Development kits | [PSOC&trade; Control C3 development kits](https://documentation.infineon.com/psoccontrolc3/docs/yyw1732688626489)
Tools, BSPs, libraries, and code examples | [ModusToolbox&trade;](https://documentation.infineon.com/modustoolbox/) – ModusToolbox&trade; software is a collection of easy-to-use libraries and tools enabling rapid development with Infineon MCUs for applications ranging from wireless and cloud-connected systems, edge AI/ML, embedded sense and control, to wired USB connectivity using PSOC&trade; Industrial/IoT MCUs, AIROC&trade; Wi-Fi and Bluetooth&reg; connectivity devices, XMC&trade; Industrial MCUs, and EZ-USB&trade;/EZ-PD&trade; wired connectivity controllers. ModusToolbox&trade; incorporates a comprehensive set of BSPs, HAL, libraries, configuration tools, and provides support for industry-standard IDEs to fast-track your embedded application development

<br>


## Other resources

Infineon provides a wealth of data at [www.infineon.com](https://www.infineon.com) to help you select the right device, and quickly and effectively integrate it into your design.


## Document history

Document title: *CE238774* – *PSOC&trade; Control C3 MCU: Trigger*

 Version | Description of change
 ------- | ---------------------
 1.0.0   | New code example
 1.1.0   | Added KIT_PSC3M6_EVAL support
 2.0.0   | Migrate CE to KIT_PSC3M5_EVK 3.x 
<br>


All referenced product or service names and trademarks are the property of their respective owners.

The Bluetooth&reg; word mark and logos are registered trademarks owned by Bluetooth SIG, Inc., and any use of such marks by Infineon is under license.

PSOC&trade;, formerly known as PSoC&trade;, is a trademark of Infineon Technologies. Any references to PSoC&trade; in this document or others shall be deemed to refer to PSOC&trade;.

---------------------------------------------------------

(c) 2026, Infineon Technologies AG, or an affiliate of Infineon Technologies AG. All rights reserved.
This software, associated documentation and materials ("Software") is owned by Infineon Technologies AG or one of its affiliates ("Infineon") and is protected by and subject to worldwide patent protection, worldwide copyright laws, and international treaty provisions. Therefore, you may use this Software only as provided in the license agreement accompanying the software package from which you obtained this Software. If no license agreement applies, then any use, reproduction, modification, translation, or compilation of this Software is prohibited without the express written permission of Infineon.
<br>
Disclaimer: UNLESS OTHERWISE EXPRESSLY AGREED WITH INFINEON, THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, ALL WARRANTIES OF NON-INFRINGEMENT OF THIRD-PARTY RIGHTS AND IMPLIED WARRANTIES SUCH AS WARRANTIES OF FITNESS FOR A SPECIFIC USE/PURPOSE OR MERCHANTABILITY. Infineon reserves the right to make changes to the Software without notice. You are responsible for properly designing, programming, and testing the functionality and safety of your intended application of the Software, as well as complying with any legal requirements related to its use. Infineon does not guarantee that the Software will be free from intrusion, data theft or loss, or other breaches (“Security Breaches”), and Infineon shall have no liability arising out of any Security Breaches. Unless otherwise explicitly approved by Infineon, the Software may not be used in any application where a failure of the Product or any consequences of the use thereof can reasonably be expected to result in personal injury.
