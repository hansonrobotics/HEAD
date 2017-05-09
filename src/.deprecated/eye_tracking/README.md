# Eye Tracking
This package modifies PAU messagee with eye angles that are calculated based on camera in the eye settings.
In the config FOV of the camera and the center point is needeed

### Config
Sample config below.
```
# Eye tracking
eye_tracking:
    angles:
        w: 0.32 # Angle (rad) which will be proportional to full height of image from camera feed and added to PAU
        h: 0.15 # Angle (rad) which will be proportional to full width of image from camera feed and added to PAU
    center: # relative camera image point which represents point where eyeball is looking at
        w: 0.27
        h: 0.72
    distance-max: 0.3 # Relative distance from camera image center to face target in which face considered to be same.
```

### Todo
Currently this module uses one camera in the eye and updates both eyes angles. 
For both eye cameras need to extend so it eye values would be corrected separately and eye_cameras names should be passed as well.

Also needs the service which allows to control which part of the face eyes should be servoing, this shpuld be passed by procedureal animations seccades.
