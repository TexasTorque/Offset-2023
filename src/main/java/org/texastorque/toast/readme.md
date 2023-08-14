# TEXAS TOAST (Texas Torque Optical Awareness System™)

Unified Vision system for the 2024 season.

## Packages

TOAST is split up into multiple packages. 
* The vision processing system (VPS) that runs on the coprocessor.
* The robot code library (ToastLib) which interfaces with roborio code.
* The web client for interfacing, debugging, and checking status. *(WIP)*

## Vision Processing System (VPS) Overview

### Dispatcher

The dispatcher, the main program is at root level, and handles the dispatching of pipelines. This is written in Python. The following files are part of the dispatcher.

| File    | Description    |
| --- | --- |
| `toast.py`	| Main program dispatcher, launches camera threads |
| `geometry.py` | Python side geometry library |
| `intrinsics.py` | Python intrinsics database library |
| `network.py`	| Python side networking library |

### Pipelines (Python component)

Pipelines have one Python file, used for processing an input and push their results to the network, and one Java class, used for reading from the network and interfacing with the robot code. The following are existing pipelines Python components. They are located in the `pipelines/` directory.

| File    | Description    |
| --- | --- |
| `pipelines/sample.py`	| Example pipeline |
| `pipelines/april-tags.py` | April Tags pipeline |
| `pipelines/cone-detection.py` | Cone detection pipeline |

### Calibration Utility

The calibration utility is used to calibrate cameras, and produce both intrinsic values and distortion constants for different camera lenses. The main calibration utility program is `cal-cam.py`, located at the root directory. It relies on `intrinsics.json` for storing these values, and the `cal-imgs/` directory for storing calibration images.

## Client Library (ToastLib)

ToastLib is a fairly complex library, but provides for low coupling and a simple interface.

**More docs coming soon.**

## Errors

If upon booting you are greeted by a terminal and not the regular Ubuntu GUI, run 
```
fsck /dev/nvme0n1p4
```
then 
```a```
then reboot.

Then run 
```sudo update-grub```
to ensure that on boot, the Mini PC will launch into Ubuntu immediately.

