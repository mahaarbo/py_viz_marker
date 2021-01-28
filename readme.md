# Simple RVIZ markers through services
This `rospy` node continuously publishes rviz markers at a particular rate (default=50).
The namespace parameter (default="") specifies the namespace underwhich the `add_marker` and `remove_marker` services are to be placed.

## Example usage
With rviz up and running and visualizing MarkerArray at `/visualization_marker_array`:

1. `rosrun py_viz_marker vizhandler.py -namespace my_viz -rate 50`
2.`rosservice call /add_marker '{label: 'test', marker: {header: {frame_id: 'base_link'}, id: 0, type: 1, action: 0, scale: {x: 2.0, y: 1.0, z: 1.0}, color: {r: 1.0, g: 0.0, b: 1.0, a: 1.0}}}'`

## Note
Internally we use the labels as ID's for the markers. This means that you can overwrite a marker by setting the same label, or remove it by referencing the label.

Inspired by [viz_marker](https://gitlab.mech.kuleuven.be/rob-expressiongraphs/viz_marker) by Erwin Aertbeliën. 
