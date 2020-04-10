# uav_gazebo

Plugin for simple simulation of a quad-copter in Gazebo and ROS.

## An example for the impatients

Start Gazebo (with ROS connection):
```
roslaunch gazebo_ros empty_world.launch
```

Spawn a drone in Gazebo, enable position/yaw control and move it around
(remember to make `example_control` executable with `chmod +x example_control`):
```
roslaunch uav_gazebo spawn.launch
rostopic pub -1 /drone/switch_mode uav_gazebo_msgs/ControlMode "mode: 1"
rosrun uav_gazebo example_control
```

## Packages

- `uav_gazebo`: catkin package containing the actual Gazebo plugin plus some example code.
- `uav_gazebo_msgs`: catkin package defining custom messages used by the plugin.

## Plugin description

The plugin is rather simple. The drone is assumed to be a floating rigid body, which can be moved around by apllying a generic torque and a force aligned to the z-axis of the body. The plugin can be attached to a model in the usual way, and will use the root link as the body that fully represents the drone.

### Dynamic parameters

The plugin will automatically collect the mass and inertia of the rigid body it is attached to, and use them as parameters for the dynamic model. There is however one restriction: the center of mass of the link must coincide with the origin of the link itself. This is because in the controllers, translational and rotational dynamics are considered to be decoupled. Note that in principle the reference frame of the inertia needs not to be aligned with the link frame, although this scenario has not be tested.

### Control modes

The plugin allows to control the drone in different control modes (they are discussed more in details in the [dedicated section in the theoretical background](#control-schemes)):

- Position and yaw
- Velocity and yaw rate
- Thrust and attitude
- Thrust and angular velocity
- Thrust and torque

In addition, the drone can be set in "idle mode". Note that the drone starts in this mode.

### ROS interface

To receive control inputs, the drone is connected with several ROS topics. In addition, it will provide some feedback on its state via some publishers. All ROS connections are "located" in a namespace that can be provided to the plugin via an XML tag, `rosNamespace`. It is recommended to set such namespace to something unique, such as the name of the drone, to allow control of multiple models at the same time.

#### Published topics

- `odometry` (type [`nav_msgs/Odometry`](http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html)): used to provide the current pose and twist of the drone. Note that the covariances are not set.
- `tf`: the plugin will publish the pose of the link it is attached to using a [`tf2::TransformBroadcaster`](http://wiki.ros.org/tf2). The reference frame id is manually set to `world` (in future versions, it might be interesting to let the user set it using an XML tag).

#### Subscribed topics

- `switch_mode` (type `uav_gazebo_msgs/ControlMode`): publish on this topic to change the control scheme used by the drone. Note that, as soon as the mode is switched, the controller will use the last available commands received for that mode (if no inputs had been received for that mode, they will default to zero). It is thus recommended to send at least one command *before* performing the switch, to make sure that the drone will not use outdated commands.
- `<control-mode>/command` (types `uav_gazebo_msgs/<ControlMode>Control`): set of topics used to receive control inputs. Each topic uses a specific message type from the `uav_gazebo_msgs` package from this repository.

#### Dynamic reconfiguration

The plugin provides a [dynamic reconfigure server](http://wiki.ros.org/dynamic_reconfigure) to change the gains of the different control layers. The server is started under the child namespace `gains`.

Note that for second order (PD) controllers the parameters that can be tuned via the dynamic reconfigure server are not the proportional and derivative gains directly. Instead, they are the natural frequency and the damping coefficient.

In addition, the utility node `configure_controllers` is provided in case multiple drones are being simulated and the gains have to be adapted for all of them. Given a set of drone namespaces, *e.g.*, `drone1`, `drone2`, *etc.*, you can connect to all servers by running
```
rosrun uav_gazebo configure_controllers drone1 drone2 ...
```
Such node will start a reconfigure server and forward every request to each drone.

Note that by the default the servers are expected to be located in the namespaces `<drone-name>/gains`. If for any reason you need to change this, two parameters can be loaded in the private namespace of `configure_controllers`: `base_ns` and `servers_ns`. They allow to change the name of the servers to be located as `<base_ns>/<drone-name>/<servers_ns>` (note that `/` characters are added automatically when needed between the three names).

### Usage example

It should be rather easy to configure the plugin. In any case, below there is a (minimal) example of URDF:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="drone">
  <xacro:arg name="drone_name" default="drone"/>
  <xacro:property name="prefix" value="$(arg drone_name)"/>

  <link name="${prefix}">
    <visual>
      <geometry>
        <cylinder length="0.01" radius="0.07"/>
      </geometry>
      <material name="orange">
         <color rgba="0.9 0.9 0.1 1.0"/>
     </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.01" radius="0.07"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="${6.5e-4}" iyy="${6.5e-4}" izz="${1.2e-3}" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>

  <gazebo>
    <plugin name="${prefix}_plugin" filename="libuav_gazebo_plugin.so">
      <rosNamespace>${prefix}</rosNamespace>
		</plugin>
  </gazebo>
</robot>
```

Note that it accepts a xacro argument, `drone_name`, so that you can spawn multiple drones by changing their name. By namespacing each plugin in `drone_name`, ROS topics collision is prevented.



## Theoretical background

### Model of the Drone

The class of UAVs considered here is the one of quad-rotors. Their state can
be described using a 12-dimensional state composed of:
- position
- linear velocity (in world frame coordinates)
- attitude (X-Y-Z)
- angular rates

These quantities are denoted as follows:

<a href="https://www.codecogs.com/eqnedit.php?latex=\inline&space;\mathrm{\mathbf{p}}&space;=&space;\begin{bmatrix}&space;x&space;&&space;y&space;&&space;z&space;\end{bmatrix}^T" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\inline&space;\mathrm{\mathbf{p}}&space;=&space;\begin{bmatrix}&space;x&space;&&space;y&space;&&space;z&space;\end{bmatrix}^T" title="\mathrm{\mathbf{p}} = \begin{bmatrix} x & y & z \end{bmatrix}^T" /></a>

<a href="https://www.codecogs.com/eqnedit.php?latex=\inline&space;\mathrm{\mathbf{v}}&space;=&space;\begin{bmatrix}&space;v_x&space;&&space;v_y&space;&&space;v_z&space;\end{bmatrix}^T" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\inline&space;\mathrm{\mathbf{v}}&space;=&space;\begin{bmatrix}&space;v_x&space;&&space;v_y&space;&&space;v_z&space;\end{bmatrix}^T" title="\mathrm{\mathbf{v}} = \begin{bmatrix} v_x & v_y & v_z \end{bmatrix}^T" /></a>

<a href="https://www.codecogs.com/eqnedit.php?latex=\inline&space;\boldsymbol{\rho}&space;=&space;\begin{bmatrix}&space;\varphi&space;&&space;\theta&space;&&space;\psi&space;\end{bmatrix}^T" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\inline&space;\boldsymbol{\rho}&space;=&space;\begin{bmatrix}&space;\varphi&space;&&space;\theta&space;&&space;\psi&space;\end{bmatrix}^T" title="\boldsymbol{\rho} = \begin{bmatrix} \varphi & \theta & \psi \end{bmatrix}^T" /></a>

<a href="https://www.codecogs.com/eqnedit.php?latex=\inline&space;\dot{\boldsymbol{\rho}}&space;=&space;\begin{bmatrix}&space;\dot{\varphi}&space;&&space;\dot{\theta}&space;&&space;\dot{\psi}&space;\end{bmatrix}^T" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\inline&space;\dot{\boldsymbol{\rho}}&space;=&space;\begin{bmatrix}&space;\dot{\varphi}&space;&&space;\dot{\theta}&space;&&space;\dot{\psi}&space;\end{bmatrix}^T" title="\dot{\boldsymbol{\rho}} = \begin{bmatrix} \dot{\varphi} & \dot{\theta} & \dot{\psi} \end{bmatrix}^T" /></a>

The orientation of the drone frame can be obtained as a sequence of three elementary **local** rotations. In particular, we consider here the orientation matrix to be expressed by:

<a href="https://www.codecogs.com/eqnedit.php?latex=\inline&space;\mathrm{\mathbf{R}}&space;=&space;\mathrm{\mathbf{R}}_x&space;(\varphi)&space;\mathrm{\mathbf{R}}_y&space;(\theta)&space;\mathrm{\mathbf{R}}_z&space;(\psi)&space;=&space;\begin{bmatrix}\cos{\left(\psi&space;\right)}&space;\cos{\left(\theta&space;\right)}&space;&&space;-&space;\sin{\left(\psi&space;\right)}&space;\cos{\left(\theta&space;\right)}&space;&&space;\sin{\left(\theta&space;\right)}\\\sin{\left(\psi&space;\right)}&space;\cos{\left(\varphi&space;\right)}&space;&plus;&space;\sin{\left(\theta&space;\right)}&space;\sin{\left(\varphi&space;\right)}&space;\cos{\left(\psi&space;\right)}&space;&&space;-&space;\sin{\left(\psi&space;\right)}&space;\sin{\left(\theta&space;\right)}&space;\sin{\left(\varphi&space;\right)}&space;&plus;&space;\cos{\left(\psi&space;\right)}&space;\cos{\left(\varphi&space;\right)}&space;&&space;-&space;\sin{\left(\varphi&space;\right)}&space;\cos{\left(\theta&space;\right)}\\\sin{\left(\psi&space;\right)}&space;\sin{\left(\varphi&space;\right)}&space;-&space;\sin{\left(\theta&space;\right)}&space;\cos{\left(\psi&space;\right)}&space;\cos{\left(\varphi&space;\right)}&space;&&space;\sin{\left(\psi&space;\right)}&space;\sin{\left(\theta&space;\right)}&space;\cos{\left(\varphi&space;\right)}&space;&plus;&space;\sin{\left(\varphi&space;\right)}&space;\cos{\left(\psi&space;\right)}&space;&&space;\cos{\left(\theta&space;\right)}&space;\cos{\left(\varphi&space;\right)}\end{bmatrix}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\inline&space;\mathrm{\mathbf{R}}&space;=&space;\mathrm{\mathbf{R}}_x&space;(\varphi)&space;\mathrm{\mathbf{R}}_y&space;(\theta)&space;\mathrm{\mathbf{R}}_z&space;(\psi)&space;=&space;\begin{bmatrix}\cos{\left(\psi&space;\right)}&space;\cos{\left(\theta&space;\right)}&space;&&space;-&space;\sin{\left(\psi&space;\right)}&space;\cos{\left(\theta&space;\right)}&space;&&space;\sin{\left(\theta&space;\right)}\\\sin{\left(\psi&space;\right)}&space;\cos{\left(\varphi&space;\right)}&space;&plus;&space;\sin{\left(\theta&space;\right)}&space;\sin{\left(\varphi&space;\right)}&space;\cos{\left(\psi&space;\right)}&space;&&space;-&space;\sin{\left(\psi&space;\right)}&space;\sin{\left(\theta&space;\right)}&space;\sin{\left(\varphi&space;\right)}&space;&plus;&space;\cos{\left(\psi&space;\right)}&space;\cos{\left(\varphi&space;\right)}&space;&&space;-&space;\sin{\left(\varphi&space;\right)}&space;\cos{\left(\theta&space;\right)}\\\sin{\left(\psi&space;\right)}&space;\sin{\left(\varphi&space;\right)}&space;-&space;\sin{\left(\theta&space;\right)}&space;\cos{\left(\psi&space;\right)}&space;\cos{\left(\varphi&space;\right)}&space;&&space;\sin{\left(\psi&space;\right)}&space;\sin{\left(\theta&space;\right)}&space;\cos{\left(\varphi&space;\right)}&space;&plus;&space;\sin{\left(\varphi&space;\right)}&space;\cos{\left(\psi&space;\right)}&space;&&space;\cos{\left(\theta&space;\right)}&space;\cos{\left(\varphi&space;\right)}\end{bmatrix}" title="\mathrm{\mathbf{R}} = \mathrm{\mathbf{R}}_x (\varphi) \mathrm{\mathbf{R}}_y (\theta) \mathrm{\mathbf{R}}_z (\psi) = \begin{bmatrix}\cos{\left(\psi \right)} \cos{\left(\theta \right)} & - \sin{\left(\psi \right)} \cos{\left(\theta \right)} & \sin{\left(\theta \right)}\\\sin{\left(\psi \right)} \cos{\left(\varphi \right)} + \sin{\left(\theta \right)} \sin{\left(\varphi \right)} \cos{\left(\psi \right)} & - \sin{\left(\psi \right)} \sin{\left(\theta \right)} \sin{\left(\varphi \right)} + \cos{\left(\psi \right)} \cos{\left(\varphi \right)} & - \sin{\left(\varphi \right)} \cos{\left(\theta \right)}\\\sin{\left(\psi \right)} \sin{\left(\varphi \right)} - \sin{\left(\theta \right)} \cos{\left(\psi \right)} \cos{\left(\varphi \right)} & \sin{\left(\psi \right)} \sin{\left(\theta \right)} \cos{\left(\varphi \right)} + \sin{\left(\varphi \right)} \cos{\left(\psi \right)} & \cos{\left(\theta \right)} \cos{\left(\varphi \right)}\end{bmatrix}" /></a>

With this parameterization, it is possible to link the angular rates to the angular velocity (in the world frame) via the relation:

<a href="https://www.codecogs.com/eqnedit.php?latex=\inline&space;\boldsymbol{\omega}&space;=&space;\mathrm{\mathbf{D}}&space;(\varphi,\theta)&space;\dot{\boldsymbol{\rho}}&space;=&space;\begin{bmatrix}&space;1&space;&&space;0&space;&&space;\sin\theta&space;\\&space;0&space;&&space;\cos\varphi&space;&&space;-\sin\varphi\cos\theta&space;\\&space;0&space;&&space;\sin\varphi&space;&&space;\cos\varphi&space;\cos\theta&space;\end{bmatrix}\dot{\boldsymbol{\rho}}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\inline&space;\boldsymbol{\omega}&space;=&space;\mathrm{\mathbf{D}}&space;(\varphi,\theta)&space;\dot{\boldsymbol{\rho}}&space;=&space;\begin{bmatrix}&space;1&space;&&space;0&space;&&space;\sin\theta&space;\\&space;0&space;&&space;\cos\varphi&space;&&space;-\sin\varphi\cos\theta&space;\\&space;0&space;&&space;\sin\varphi&space;&&space;\cos\varphi&space;\cos\theta&space;\end{bmatrix}\dot{\boldsymbol{\rho}}" title="\boldsymbol{\omega} = \mathrm{\mathbf{D}} (\varphi,\theta) \dot{\boldsymbol{\rho}} = \begin{bmatrix} 1 & 0 & \sin\theta \\ 0 & \cos\varphi & -\sin\varphi\cos\theta \\ 0 & \sin\varphi & \cos\varphi \cos\theta \end{bmatrix}\dot{\boldsymbol{\rho}}" /></a>

Furthermore, the angular acceleration in the world frame is given by:

<a href="https://www.codecogs.com/eqnedit.php?latex=\inline&space;\dot{\boldsymbol{\omega}}&space;=&space;\mathrm{\mathbf{D}}&space;\ddot{\boldsymbol{\rho}}&space;&plus;&space;\dot{\mathrm{\mathbf{D}}}&space;\dot{\boldsymbol{\rho}}\qquad&space;\dot{\mathrm{\mathbf{D}}}&space;=&space;\begin{bmatrix}&space;0&space;&&space;0&space;&&space;\dot{\theta}\cos\theta&space;\\&space;0&space;&&space;-\dot{\varphi}\sin\varphi&space;&&space;-\dot{\varphi}\cos\varphi\cos\theta&plus;\dot{\theta}\sin\varphi\sin\theta&space;\\&space;0&space;&&space;\dot{\varphi}\cos\varphi&space;&&space;-\dot{\varphi}\sin\varphi\cos\theta-\dot{\theta}\cos\varphi\sin\theta&space;\end{bmatrix}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\inline&space;\dot{\boldsymbol{\omega}}&space;=&space;\mathrm{\mathbf{D}}&space;\ddot{\boldsymbol{\rho}}&space;&plus;&space;\dot{\mathrm{\mathbf{D}}}&space;\dot{\boldsymbol{\rho}}\qquad&space;\dot{\mathrm{\mathbf{D}}}&space;=&space;\begin{bmatrix}&space;0&space;&&space;0&space;&&space;\dot{\theta}\cos\theta&space;\\&space;0&space;&&space;-\dot{\varphi}\sin\varphi&space;&&space;-\dot{\varphi}\cos\varphi\cos\theta&plus;\dot{\theta}\sin\varphi\sin\theta&space;\\&space;0&space;&&space;\dot{\varphi}\cos\varphi&space;&&space;-\dot{\varphi}\sin\varphi\cos\theta-\dot{\theta}\cos\varphi\sin\theta&space;\end{bmatrix}" title="\dot{\boldsymbol{\omega}} = \mathrm{\mathbf{D}} \ddot{\boldsymbol{\rho}} + \dot{\mathrm{\mathbf{D}}} \dot{\boldsymbol{\rho}}\qquad \dot{\mathrm{\mathbf{D}}} = \begin{bmatrix} 0 & 0 & \dot{\theta}\cos\theta \\ 0 & -\dot{\varphi}\sin\varphi & -\dot{\varphi}\cos\varphi\cos\theta+\dot{\theta}\sin\varphi\sin\theta \\ 0 & \dot{\varphi}\cos\varphi & -\dot{\varphi}\sin\varphi\cos\theta-\dot{\theta}\cos\varphi\sin\theta \end{bmatrix}" /></a>

Finally, the dynamic of the model writes as:

<a href="https://www.codecogs.com/eqnedit.php?latex=\inline&space;\dot{\mathrm{\mathbf{v}}}&space;=&space;\frac{1}{m}\mathrm{\mathbf{f}}&space;&plus;&space;\mathrm{\mathbf{g}}&space;=&space;\frac{1}{m}\begin{bmatrix}&space;\sin\theta&space;\\&space;-\sin\varphi\cos\theta&space;\\&space;\cos\varphi\cos\theta&space;\end{bmatrix}&space;f&space;-&space;\begin{bmatrix}0&space;\\&space;0&space;\\&space;g\end{bmatrix}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\inline&space;\dot{\mathrm{\mathbf{v}}}&space;=&space;\frac{1}{m}\mathrm{\mathbf{f}}&space;&plus;&space;\mathrm{\mathbf{g}}&space;=&space;\frac{1}{m}\begin{bmatrix}&space;\sin\theta&space;\\&space;-\sin\varphi\cos\theta&space;\\&space;\cos\varphi\cos\theta&space;\end{bmatrix}&space;f&space;-&space;\begin{bmatrix}0&space;\\&space;0&space;\\&space;g\end{bmatrix}" title="\dot{\mathrm{\mathbf{v}}} = \frac{1}{m}\mathrm{\mathbf{f}} + \mathrm{\mathbf{g}} = \frac{1}{m}\begin{bmatrix} \sin\theta \\ -\sin\varphi\cos\theta \\ \cos\varphi\cos\theta \end{bmatrix} f - \begin{bmatrix}0 \\ 0 \\ g\end{bmatrix}" /></a>

<a href="https://www.codecogs.com/eqnedit.php?latex=\inline&space;\boldsymbol{\tau}&space;=&space;\mathrm{\mathbf{I}}\dot{\boldsymbol{\omega}}&space;&plus;&space;\boldsymbol{\omega}&space;\times&space;\mathrm{\mathbf{I}}\boldsymbol{\omega}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\inline&space;\boldsymbol{\tau}&space;=&space;\mathrm{\mathbf{I}}\dot{\boldsymbol{\omega}}&space;&plus;&space;\boldsymbol{\omega}&space;\times&space;\mathrm{\mathbf{I}}\boldsymbol{\omega}" title="\boldsymbol{\tau} = \mathrm{\mathbf{I}}\dot{\boldsymbol{\omega}} + \boldsymbol{\omega} \times \mathrm{\mathbf{I}}\boldsymbol{\omega}" /></a>

where the control inputs are:

<a href="https://www.codecogs.com/eqnedit.php?latex=\inline&space;f,\,\boldsymbol{\tau}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\inline&space;f,\,\boldsymbol{\tau}" title="f,\,\boldsymbol{\tau}" /></a>

*i.e.*, the thrust magnitude and the torque (expressed in the world frame). Note that in the dynamic model, the inertia is expressed in world frame coordinates. It can be obtained from the constant body-frame inertia by "rotating" it according to:

<a href="https://www.codecogs.com/eqnedit.php?latex=\inline&space;\mathrm{\mathbf{I}}&space;=&space;\mathrm{\mathbf{R}}&space;\,&space;\mathrm{\mathbf{I}}_{b}&space;\,&space;\mathrm{\mathbf{R}}^T" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\inline&space;\mathrm{\mathbf{I}}&space;=&space;\mathrm{\mathbf{R}}&space;\,&space;\mathrm{\mathbf{I}}_{b}&space;\,&space;\mathrm{\mathbf{R}}^T" title="\mathrm{\mathbf{I}} = \mathrm{\mathbf{R}} \, \mathrm{\mathbf{I}}_{b} \, \mathrm{\mathbf{R}}^T" /></a>



### Control Schemes

#### Direct control

Not much to say here, thrust and torque are directly sent to move the drone.


#### Thrust & angular velocity

The thrust is forwarded *as is*, while the torque is computed using a proportional controller (optionally with a user-supplied feedforward term). To achieve the control, the angular acceleration is computed as:

<a href="https://www.codecogs.com/eqnedit.php?latex=\inline&space;\dot{\boldsymbol{\omega}}_c&space;=&space;\dot{\boldsymbol{\omega}}^\star&space;&plus;&space;k_\omega&space;(&space;\boldsymbol{\omega}^\star&space;-&space;\boldsymbol{\omega}&space;)" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\inline&space;\dot{\boldsymbol{\omega}}_c&space;=&space;\dot{\boldsymbol{\omega}}^\star&space;&plus;&space;k_\omega&space;(&space;\boldsymbol{\omega}^\star&space;-&space;\boldsymbol{\omega}&space;)" title="\dot{\boldsymbol{\omega}}_c = \dot{\boldsymbol{\omega}}^\star + k_\omega ( \boldsymbol{\omega}^\star - \boldsymbol{\omega} )" /></a>

And the torque is then computed via the dynamic model of the drone.


#### Thrust & attitude

The thrust is forwarded *as is*. For the attitude control, a proportional and derivative pseud-control is evaluated:

<a href="https://www.codecogs.com/eqnedit.php?latex=\inline&space;\ddot{\boldsymbol{\rho}}_c&space;=&space;\ddot{\boldsymbol{\rho}}^\star&space;&plus;&space;k_{d,\rho}&space;(&space;\dot{\boldsymbol{\rho}}^\star&space;-&space;\dot{\boldsymbol{\rho}}&space;)&space;&plus;&space;k_{p,\rho}&space;(&space;\boldsymbol{\rho}^\star&space;-&space;\boldsymbol{\rho}&space;)" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\inline&space;\ddot{\boldsymbol{\rho}}_c&space;=&space;\ddot{\boldsymbol{\rho}}^\star&space;&plus;&space;k_{d,\rho}&space;(&space;\dot{\boldsymbol{\rho}}^\star&space;-&space;\dot{\boldsymbol{\rho}}&space;)&space;&plus;&space;k_{p,\rho}&space;(&space;\boldsymbol{\rho}^\star&space;-&space;\boldsymbol{\rho}&space;)" title="\ddot{\boldsymbol{\rho}}_c = \ddot{\boldsymbol{\rho}}^\star + k_{d,\rho} ( \dot{\boldsymbol{\rho}}^\star - \dot{\boldsymbol{\rho}} ) + k_{p,\rho} ( \boldsymbol{\rho}^\star - \boldsymbol{\rho} )" /></a>

The corresponding angular acceleration is then computed, and finally the torque is obtained via the dynamic model.


#### Position & yaw

The goal is now to control the position and the yaw (Z) rotation of the drone. To do this, the idea is to consider the drone as a device able to produce an orientable force. First of all, an acceleration pseudo-control is computed via a proportional and derivative controller:

<a href="https://www.codecogs.com/eqnedit.php?latex=\inline&space;\dot{\mathrm{\mathbf{v}}}_c&space;=&space;\dot{\mathrm{\mathbf{v}}}^\star&space;&plus;&space;k_{d}&space;(&space;\mathrm{\mathbf{v}}^\star&space;-&space;\mathrm{\mathbf{v}}&space;)&space;&plus;&space;k_{p}&space;(&space;\mathrm{\mathbf{p}}^\star&space;-&space;\mathrm{\mathbf{p}}&space;)" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\inline&space;\dot{\mathrm{\mathbf{v}}}_c&space;=&space;\dot{\mathrm{\mathbf{v}}}^\star&space;&plus;&space;k_{d}&space;(&space;\mathrm{\mathbf{v}}^\star&space;-&space;\mathrm{\mathbf{v}}&space;)&space;&plus;&space;k_{p}&space;(&space;\mathrm{\mathbf{p}}^\star&space;-&space;\mathrm{\mathbf{p}}&space;)" title="\dot{\mathrm{\mathbf{v}}}_c = \dot{\mathrm{\mathbf{v}}}^\star + k_{d} ( \mathrm{\mathbf{v}}^\star - \mathrm{\mathbf{v}} ) + k_{p} ( \mathrm{\mathbf{p}}^\star - \mathrm{\mathbf{p}} )" /></a>

which could be achieved if the force vector

<a href="https://www.codecogs.com/eqnedit.php?latex=\inline&space;\mathrm{\mathbf{f}}_c&space;=&space;m\dot{\mathrm{\mathbf{v}}}_c&space;-&space;m\mathrm{\mathbf{g}}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\inline&space;\mathrm{\mathbf{f}}_c&space;=&space;m\dot{\mathrm{\mathbf{v}}}_c&space;-&space;m\mathrm{\mathbf{g}}" title="\mathrm{\mathbf{f}}_c = m\dot{\mathrm{\mathbf{v}}}_c - m\mathrm{\mathbf{g}}" /></a>

was applied to the drone. By comparison with the drone dynamic model, such force can be produced if

<a href="https://www.codecogs.com/eqnedit.php?latex=\inline&space;\mathrm{\mathbf{f}}_c&space;=&space;\begin{bmatrix}&space;\sin\theta&space;\\&space;-\sin\varphi\cos\theta&space;\\&space;\cos\varphi\cos\theta\end{bmatrix}&space;f" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\inline&space;\mathrm{\mathbf{f}}_c&space;=&space;\begin{bmatrix}&space;\sin\theta&space;\\&space;-\sin\varphi\cos\theta&space;\\&space;\cos\varphi\cos\theta\end{bmatrix}&space;f" title="\mathrm{\mathbf{f}}_c = \begin{bmatrix} \sin\theta \\ -\sin\varphi\cos\theta \\ \cos\varphi\cos\theta\end{bmatrix} f" /></a>

Solving for the two angles and the force, one obtains:

<a href="https://www.codecogs.com/eqnedit.php?latex=\inline&space;f&space;=&space;\Vert&space;\mathrm{\mathbf{f}}_c&space;\Vert&space;\quad&space;\varphi^\star&space;=&space;\mathrm{atan2}\left(&space;-f_y,&space;f_z&space;\right)&space;\quad&space;\theta^\star&space;=&space;\mathrm{asin}&space;\left(&space;f_x&space;/&space;f&space;\right)" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\inline&space;f&space;=&space;\Vert&space;\mathrm{\mathbf{f}}_c&space;\Vert&space;\quad&space;\varphi^\star&space;=&space;\mathrm{atan2}\left(&space;-f_y,&space;f_z&space;\right)&space;\quad&space;\theta^\star&space;=&space;\mathrm{asin}&space;\left(&space;f_x&space;/&space;f&space;\right)" title="f = \Vert \mathrm{\mathbf{f}}_c \Vert \quad \varphi^\star = \mathrm{atan2}\left( -f_y, f_z \right) \quad \theta^\star = \mathrm{asin} \left( f_x / f \right)" /></a>

which can be forwarded to the thrust and attitude control described above. Note that the yaw is regulated to the desired value by the attitude controller.


#### Velocity & yaw rate

This control mode consists in nothing but a slightly modified version of the position and yaw controller. In this case, one can imagine to set the proportional gain of the position controller to zero (in this way, only the desired velocity will be regulated). Similarly, in the attitude controller one can still regulate the first two angles to the values produced by the velocity controller (to properly orient the thrust direction), while for the yaw regulation it suffices to set the proportional gain to zero (so that only the yaw rate will be taken into account).
