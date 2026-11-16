# PSoC Edge E84 Air Quality Firmware

Firmware for an  smoking-detection research prototype on the Infineon **KIT_PSE84_AI**. It combines a Sensirion SEN66 environmental sensor, XENSIV radar, USB camera inference, a local display, and Bluetooth Low Energy (BLE). The [PSE84 IAQ Monitor app](https://github.com/Jifsopeon/BLE_APP/tree/master) displays the environmental stream and records labelled samples.

## What runs on each core

| Image | Responsibility |
| --- | --- |
| CM33 Secure | Initializes the secure platform and hands off to the non-secure application. |
| CM33 Non-Secure | Reads the SEN66 over I²C, processes radar presence/distance, maintains the manual label, runs the deployed environmental decision tree, and publishes an IAQ stream over BLE. It also starts CM55. |
| CM55 | Acquires USB camera frames, runs the bundled image model, renders the camera preview and detection boxes on the display, and reports events to CM33 through shared memory. |

The **environmental prediction** and **camera detection** are separate inference paths. Radar distance is streamed alongside the environmental readings, but is not an input to the bundled environmental tree. Likewise, the BLE app's **Predicted** field comes from the environmental stream, not from bounding boxes on the board display. The code does not send camera frames over this BLE interface.

## Hardware and tools

- Infineon **KIT_PSE84_AI** and a data-capable USB cable for KitProg3.
- Sensirion **SEN66** connected to the board's configured I²C bus. The driver uses the BSP's `CYBSP_I2C_CAM_CONTROLLER` configuration. Consult the kit and sensor hardware documentation before connecting power and signals.
- A compatible USB webcam and MIPI display used by the AI-kit CM55 configuration for the vision demo. The default camera build is `CAM_USB`. A camera must expose the frame format and size selected by the firmware.
- ModusToolbox 3.7 toolchain baseline, Git, and a serial terminal. The checked-in [build configuration](common.mk) selects `APP_KIT_PSE84_AI` and `GCC_ARM`.

## Build and program on Windows

Connect the board through the **KitProg3** USB port. Open **modus-shell** and run the following from the workspace root:

```sh
git clone --branch master https://github.com/Jifsopeon/kit_pse84_ai_iaq.git
cd kit_pse84_ai_iaq
make getlibs
make build
make program
```

Build and program from the root so ModusToolbox handles all three images and the combined output. The root [Makefile](Makefile) sets `SKIP_CODE_GEN=1` because this project carries checked-in generated configuration for the CM55 display and ML integration. Keep that setting for a normal build. Do not regenerate or replace the BSP configuration just to build the demo. For vendor IDE and programming guidance, see [Using the code example](docs/using_the_code_example.md).

## Run the demo

1. Open the KitProg3 COM port at **115200 baud, 8 data bits, no parity, 1 stop bit**. Reset the kit if you want to capture the startup messages.
2. Check the serial stream for SEN66 readings. The environmental task defaults to one sample per second.
3. With the webcam and display connected, check the local preview and the detection overlay. If there is no preview, check USB camera compatibility and the CM55 camera/display diagnostics first.
4. In the companion app, scan for **PSE84-IAQ** and connect. Check that the readings and charts update. **SW1** on the board toggles the manual smoking label. The app can also request a label change over BLE. Keep this label separate from the model prediction when collecting data.

## Source map

| Path | What to look for |
| --- | --- |
| [`proj_cm33_s/`](proj_cm33_s/) | Secure application and boot handoff. |
| [`proj_cm33_ns/main.c`](proj_cm33_ns/main.c) | FreeRTOS tasks, SEN66 and radar integration, serial output, and CM55 launch. |
| [`proj_cm33_ns/source/prediction/`](proj_cm33_ns/source/prediction/) and [`source/model/`](proj_cm33_ns/source/model/) | Environmental feature history and the embedded decision tree. |
| [`proj_cm33_ns/source/app_ble_iaq.c`](proj_cm33_ns/source/app_ble_iaq.c) and [`GeneratedSource/`](proj_cm33_ns/GeneratedSource/) | IAQ BLE service and generated GAP/GATT configuration. |
| [`proj_cm55/source/`](proj_cm55/source/) and [`proj_cm55/model/`](proj_cm55/model/) | USB camera, vision inference, display tasks, and bundled image model. |
| [`components/COMPONENT_SHARED_MEM/`](components/COMPONENT_SHARED_MEM/) | Shared-memory event interface. |
| [`bsps/TARGET_APP_KIT_PSE84_AI/`](bsps/TARGET_APP_KIT_PSE84_AI/) | Board configuration, peripheral assignments, and linker files. |

The SEN66 training scripts and five-model environmental comparison are maintained separately in the **Definitive Evaluation** archive. The vision model is not one of those five models. This is a research prototype, not a certified smoke or fire alarm.

The workspace builds on Infineon's [PSoC Edge empty application example](https://github.com/Infineon/mtb-example-psoc-edge-empty-app). See the repository [LICENSE](LICENSE) and retain the component-specific notices in the source and BSP.
