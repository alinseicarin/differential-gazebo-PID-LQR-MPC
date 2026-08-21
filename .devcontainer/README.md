# Development container configuration

`devcontainer.json` is intentionally kept as strict JSON, so its settings are
documented here rather than with inline comments.

- `build` constructs the sibling `Dockerfile` with an `ubuntu` development
  user. The Dockerfile defaults to UID/GID 1000 so bind-mounted files normally
  retain the host user's ownership.
- `workspaceFolder` and `workspaceMount` bind the repository into `/home/ws`.
  This path explains why older controller code used `/home/ws` explicitly;
  current launch files instead locate installed assets through the ament index.
- `--net=host` lets ROS 2 DDS discovery and Gazebo networking use the host
  network directly.
- `--ipc=host` and `--pid=host` share IPC and process namespaces with the host.
- `--privileged` and the `/dev/dri` mount permit hardware-accelerated Gazebo and
  RViz rendering. Privileged containers have broad host access and should be
  used only as a trusted local development environment.
- The display, Wayland, runtime-directory, and PulseAudio variables forward GUI
  and audio configuration from the host.
- `/mnt/wslg`, `/tmp/.X11-unix`, and `/dev/dri` expose WSLg/X11 sockets and GPU
  devices required by Linux GUI applications.
- The VS Code extensions provide C++, Python, and ROS language/tool support.
- `remoteUser` ensures terminals and editor operations run as `ubuntu`, not
  root.
- `postCreateCommand` initializes rosdep when necessary, updates its package
  index, and installs dependencies declared by packages under `src`.

After creating or rebuilding the container, build the overlay with:

```bash
cd /home/ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```
