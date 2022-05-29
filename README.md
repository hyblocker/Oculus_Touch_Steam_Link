# OculusTouchLink
## Turn Oculus Rift controllers into emulated Vive trackers!

This only works with the Rift CV1:
(This one)

![oculus rift](https://raytracing-benchmarks.are-really.cool/5n6WJQv.png)

## Downloading:
**Getting the package:**  
First you're going to need the main release package available. [Click here to download it](https://github.com/hyblocker/Oculus_Touch_Steam_Link/raw/main/ReleasePackage/OculusTouchLink.zip)

If you have an **Oculus Rift S** You won't be able to use this because the Oculus app is occupied by the Rift.

If you have an **Oculus Quest or Quest 2**, you must use something like [ALVR](https://github.com/alvr-org/alvr) or Virtual Desktop instead of Link or Air Link for the same reason as above.

```
Virtual Desktop emulates an Oculus device and uses the SteamVR Oculus driver,
so this may cause some extra issues compared to using ALVR.
```

**Getting OpenVR-SpaceCalibrator:**  
You must [download it from here](https://github.com/pushrax/OpenVR-SpaceCalibrator) in the **Releases** section on the right.

Run the EXE and install it.

## Configuring trackers:
By default, TouchLink will spawn 3 trackers for the Left Foot, Left Leg and Waist. But you can create files in `%localappdata%\TouchLink` to stop individual trackers from spawning.
- Open the Windows run Dialog by pressing <kbd>Win</kbd> + <kbd>R</kbd>
- Put in `%localappdata%` and press <kbd>Enter</kbd>.
- In the file explorer window, create a new folder and name it `TouchLink`
![right-click menu](https://raytracing-benchmarks.are-really.cool/55eU5f4.png)
- **Make sure you have file extensions enabled.**
![enabling file extensions](https://raytracing-benchmarks.are-really.cool/gGj4qJE.png)
- Rename the file, including the `.txt` part to either `no-hip`, `no-left-leg` or `no-right-leg`.

## Setting it up:
**Copying things in the right places:**  
Extract the `OculusTouchLink.zip` file using File Explorer into a folder of your choosing.

Access the SteamVR folder

![opening steamvr local files](https://raytracing-benchmarks.are-really.cool/Af6eSnS.png)

Go to the `drivers` folder and create a new folder with whatever name you want. maybe `OculusTouchLink`

Copy the files and folders
```
bin
resources
driver.vrdrivermanifest
```
into that newly created folder.

*Alternatively if you know what you're doing add it to `openvrpaths.vrpath`*

## Starting it up:
Start up the Oculus Rift then launch the file named `ovr_test`. It will show a command line window with tracking updates from your Oculus controllers.

If you want the headset to not go to sleep after 5 minutes, **causing the trackers to freeze**, either attach it to a rotating fan, or put a soda can in front of the proximity sensor.

From here, you can start SteamVR as usual, the OculusTouchLink driver should see the data coming from `ovr_test` and create the trackers for you. **But they're not calibrated!**

## Calibration:
In the SteamVR dashboard, in the belt at the bottom should be an icon for OpenVR Space Calibrator. Click on it. Pick the tracking space of your headset on the left, and the tracking space of your Oculus Rift controller tracker monsters on the right.

Select a device in each column and grab them together in your hand, then click short calibration.

Move the two controllers together in an infinity 8 shape until the timer is finished.

If the calibration isn't good enough, try holding the controllers together more tightly, or use a longer calibration.

**You only have to calibrate once!**

If this last step is confusing you, you can watch any video for using Vive trackers with Oculus, and the OpenVR-SpaceCalibrator steps will be the same.

Every game will recognize the trackers as if they were real Vive trackers, so no extra setup is required.

## Thanks
https://github.com/mm0zct/Oculus_Touch_Steam_Link for the original app, all the real magic was done here

https://github.com/TripingPC/Oculus_Touch_Steam_Link for forking the original app, and making it spawn Vive trackers, and providing the original readme!

https://github.com/AeroScripts/Oculus_Touch_Steam_Link for forking Aurora's fork and adding support for the third controller, that's really cool!

Mark Zuckerberg, for flooding the market with cheap broken second-hand Rift headsets.