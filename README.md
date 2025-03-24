# Human Follower

The Human Follower package is a ROS2-based project designed to detect and follow a human using a laser scanner. It consists of two main nodes: `human_detector` and `human_follower`.

## Package Structure

```
human_follower/
├── __init__.py
├── human_detector.py
├── human_detector1.py
├── human_follower.py
resource/
├── human_follower
test/
├── test_copyright.py
├── test_flake8.py
├── test_pep257.py
package.xml
setup.cfg
setup.py
```

## Nodes

### Human Detector

The `human_detector` node subscribes to laser scan data, processes it to detect human legs, and publishes the detected human position.

#### Subscriptions

- `/scan` (`sensor_msgs/LaserScan`): Laser scan data.

#### Publications

- `/human_position` (`geometry_msgs/PointStamped`): Detected human position in the laser frame.
- `/human_pose` (`geometry_msgs/PoseStamped`): Detected human pose in the base_link frame.
- `/human_marker` (`visualization_msgs/Marker`): Visualization marker for the detected human.

### Human Follower

The `human_follower` node subscribes to the detected human pose and sends navigation goals to follow the human while maintaining a safe distance.

#### Subscriptions

- `/human_pose` (`geometry_msgs/PoseStamped`): Detected human pose in the base_link frame.

#### Action Clients

- `navigate_to_pose` (`nav2_msgs/action/NavigateToPose`): Sends navigation goals to the Nav2 stack.

## Requirements

- ROS2 Foxy or later
- Nav2 (Navigation2) stack

## Installation

1. Clone the repository:
    ```sh
    git clone <repository_url>
    ```

2. Navigate to the package directory:
    ```sh
    cd human_follower
    ```

3. Install dependencies:
    ```sh
    rosdep install --from-paths src --ignore-src -r -y
    ```

4. Build the package:
    ```sh
    colcon build
    ```

## Running the Nodes

1. Source the setup script:
    ```sh
    source install/setup.bash
    ```

2. Ensure the Nav2 stack is running:
    ```sh
    ros2 launch nav2_bringup navigation_launch.py
    ```

3. Run the `human_detector` node:
    ```sh
    ros2 run human_follower human_detector_node
    ```

4. Run the `human_follower` node:
    ```sh
    ros2 run human_follower human_follower
    ```

## Testing

To run the tests, use the following command:
```sh
colcon test
```

## License

TODO: License declaration

## Maintainer

muhammedsiyadp - muhammedsiyadofficial@gmail.com