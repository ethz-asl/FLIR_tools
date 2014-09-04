FLIR_tools
==========

Software for FLIR thermal camera interfacing.

ir_viewer
----------

Tool for displaying and converting false colored infrared images. 

### Display image 
```
rosrun ir_viewer ir_viewer image:=<image topic> [image transport type]
```

### Convert image to fales color
```
rosrun ir_viewer ir_converter image:=<image topic> [image transport type]
```
