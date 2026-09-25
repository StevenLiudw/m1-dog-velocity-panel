# M1 dog velocity control panel

A browser panel for sending body velocity commands to the M1 robot dog. Run the
panel on a computer connected to the dog's network, choose a robot mode, and
command forward, sideways, and yaw velocity. The page listens on
`http://127.0.0.1:8765/` on **your computer**.

The panel uses an SSH session to start a small DDS publisher on the dog. That
publisher sends `rt/ltr/sportmode_cmd` and `rt/ltr/vel_cmd` to the existing
onboard controller. It does not replace or restart `dg_fsm.service`, and the
computer does not need ROS or the robot SDK for normal use.

## Quick start

Prerequisites:

- A computer that can reach the dog over SSH. Get its SSH address and account
  from the robot operator; they are not included in this public repository.
- Python 3, OpenSSH (`ssh`), and `script` from `util-linux` on that computer.
- An SSH account on the dog and the auxiliary `dog_dds_bridge` executable
  installed at `~/dg_fsm_m1/build/test_repo/dog_dds_bridge` on that account.
- The G20 remote available to take over control if needed.

```bash
git clone https://github.com/StevenLiudw/m1-dog-velocity-panel.git
cd m1-dog-velocity-panel
./run.sh --robot USER@ROBOT_IP
```

Open <http://127.0.0.1:8765/> on the same computer. Enter the robot SSH
password in the panel and click **Connect**. The password is sent to the local
server for this SSH login and is not saved. Do not put it in a command line,
configuration file, or Git commit.

You can also set `DOG_ROBOT_SSH_TARGET=USER@ROBOT_IP` in your shell. To change
the local port:

```bash
./run.sh --robot USER@ROBOT_IP --port 8766
```

## Control sequence

1. Put the dog in a clear area and keep the G20 remote ready. Hold its **HOME**
   switch to select HIGHLEVELSDK/DDS control. Releasing HOME returns control
   to the gamepad.
2. In the panel, select **Stand up**, wait for a stable stance, then select
   **Walk**.
3. Click **Arm velocity**. This enables the panel's movement controls; it is
   not a robot mode and does not itself move the dog.
4. For directional control, hold a panel arrow or `W/A/S/D/Q/E` key. Releasing
   sends zero velocity. For an exact command, enter `vx`, `vy`, and `wz`, then
   click **Start entered velocity**. Click **Stop entered velocity** to send
   zero. Changing a field while running takes effect after stopping and
   starting again.
5. Press **STOP · Disarm** or Space to send zero and disarm. Changing robot
   mode, leaving the tab, or losing browser focus also stops the command.

`vx` is forward speed in m/s, `vy` is leftward speed in m/s, and `wz` is yaw
rate in rad/s. Limits accepted by this panel match the deployed subscriber:
`vx ±2.0`, `vy ±0.6`, and `wz ±0.8`. Start with a low velocity. The black
readout shows the command the panel is currently sending, **not** the values
merely typed into the fields or the dog's measured speed.

## How the connection works

```text
Browser on 127.0.0.1:8765
  → Python panel server on the same computer
  → persistent SSH session to the dog
  → auxiliary DDS publisher on the dog's loopback interface
  → onboard dg_fsm.service subscriber and locomotion controller
```

The browser refreshes active velocity commands every 100 ms. The Python
server stops and disarms after 400 ms without a new command, and the
robot-side publisher sends zero after 350 ms without a new command. Closing
the browser or losing the SSH connection should therefore stop the auxiliary
publisher's velocity stream. The deployed motor controller has no confirmed
DDS velocity timeout of its own, so keep the G20 remote available.

Mode selection and the black readout report **commands sent by the panel**.
The panel has no robot state telemetry or acknowledgement that the dog changed
mode or physically moved. A zero velocity reached `VelCmdHandler` on the
current dog, and a Stand command reached `SportModeCmdHandler`; nonzero
physical movement has not been verified from this panel.

## If the dog does not respond

| Panel observation | Check |
| --- | --- |
| DDS publisher disconnected | Verify the dog is reachable over SSH, enter the SSH password, and click Connect. |
| Mode button is selected, but the dog does not change mode | Keep the G20 HOME switch held and check the dog is using HIGHLEVELSDK control. A selected button is not robot state telemetry. |
| Exact fields show numbers, but black readout is zero | Select Walk, click Arm velocity, then Start entered velocity. Typing alone does not send a command. |
| Black readout is nonzero, but the dog does not move | Stop the command. Check the G20 control selection, robot stance, and onboard controller logs; the readout confirms only the panel's outgoing command. |

The current robot service started its DDS reader when `eth0` was down.
Publishing directly from the computer did not reach it. The default SSH
transport runs the auxiliary publisher on the dog's loopback interface, which
reached the subscriber without restarting the service.

## Install or rebuild the robot-side publisher

The quick start works only when the auxiliary publisher is already installed
on the dog. To rebuild it, the robot needs its LTR DDS SDK checkout at
`~/dg_fsm_m1`, a C++17 compiler, and the bundled DDS libraries. The
deployment script copies this repository's `dds_bridge.cpp`, builds it on
the robot, and installs it at the path used by the panel:

```bash
DOG_ROBOT_SSH_TARGET=USER@ROBOT_IP ./deploy_robot_bridge.sh
```

The build paths in the script match the current M1 installation and
need adjustment if another robot has a different SDK layout. Deployment does
not restart `dg_fsm.service`.

For a direct host DDS setup after the robot's network startup is repaired,
set `DOG_SDK_ROOT` to a local LTR SDK checkout and run:

```bash
./build.sh
python3 server.py --transport local --iface YOUR_DOG_NETWORK_INTERFACE
```

## Rehearse without robot commands

```bash
python3 server.py --simulate --port 8766
```

Open <http://127.0.0.1:8766/>. Simulation shows the panel's mode and
velocity flow without opening SSH or publishing DDS messages.
