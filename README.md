Description to be written!

## Example

Start Gazebo:
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

*i.e.*, the thrust magnitude and the torque (expressed in the world frame).



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
