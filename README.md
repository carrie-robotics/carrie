# <img src="docs/carrie-icon1-512x512.png" width="24" height="24" alt="icon" /> carrie
Building a mobile manipulator based on a [Fetch Mobile Manipulator](https://github.com/ZebraDevs/fetch_ros) chassis.
<p align="center">
  <img src="docs/fetch13_chassis1.jpeg" alt="fetch 13 chassis" width="400" />
</p>

# tl;dr
Build a new development image

> [!Note]
> The `REPO` variable is defined in `.env`

```
source .env
mkdir -p ~/.${REPO}/ccache
export UID=$(id -u) GID=$(id -g); docker compose -f compose.dev.yml build
```
Start an interactive development container
```
docker compose -f compose.dev.yml run --rm development
```
Build the repository in the container
```
colcon build
```

# test
To test that your container is working with graphics
```shell
colcon build
source install/setup.bash
ros2 launch carrie_description view_carrie.launch.py
```

# orphans
```shell
docker compose -f compose.dev.yml down --remove-orphans
```
