This is an example implementation of a LUT based segmentation module that can be used with Realsense cameras.

Designed to use Python 3. Dependencies: numpy, openCV, pyrealsense2

This code is heavily inspired by [Mitupead - football robot](https://github.com/lwd8cmd/Mitupead)

## How to use

Segmentation module installation:
```
cd segment_module
pip3 install .
```

Running color configurator:
```
mkdir colors
touch colors/colors.pkl
python3 config_colors.py
```

Running an example:
```
python3 main.py
```

If you encounter dependency errors, resolve them with pip3


Code repositories must include a README describing how to set up and run the code
