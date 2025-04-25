# Custom Service and Client

## Objectives

- You will create a service with a new definition to apply it on the screen.
- You will create a battery node that will contain a battery state.
- You will also create a LED panel node which will contain the state of three LEDs and will print every second this state. So you don’t need to real LEDs for this activity.
- You will have to create a new service definition with one request containing the number of the LED and another number 0 or 1 to power off and on the LED. The response will simply contain a boolean flag to notify if the state of the LED could be successfully changed.
- When the battery is empty, you will send a request to the sever to ask for one LED to be powered on and when the batter is full, you will power off the LED. As you have no real battery here, you can just fake the battery state.
- Let’s say that the battery will switch from a full state to an empty state every seven seconds, and then will switch back to a full state in three seconds.

## Create a new service definition

- Create a new service definition in the `srv` folder of your package. The name of the service should be `SetLED.srv`.
- The request should contain two integers: the number of the LED (0, 1, or 2) and the state of the LED (0 for off and 1 for on).
- The response should contain a boolean flag to notify if the state of the LED could be successfully changed.

```python
int64 led_number
int64 state
---
bool success
```

## Update the package.xml file

- Include the necessary dependencies in the `package.xml` file.

- Add the following lines to the `package.xml` file:

```xml
<build_depend>message_generation</build_depend>
<exec_depend>message_runtime</exec_depend>
```

## Update the CMakeLists.txt file

- Add the following lines to the `CMakeLists.txt` file:

```cmake
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

add_service_files(
  FILES
  SetLED.srv
)

generate_messages(
  DEPENDENCIES std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime rospy std_msgs
)
```

## Create the Battery Node

- Create a file named battery.py in the `src` folder of the `exercises` package.

```bash
cd ~/catkin_ws/src/exercises/src
touch battery.py
```

- Open the file and add the following code:

```python
#!/usr/bin/env python
import rospy
from exercises.srv import SetLED
from rospy import Timer, Duration
from std_msgs.msg import Bool

class Battery:
    def __init__(self):
        rospy.init_node('battery_node')
        self.is_full = True
        rospy.loginfo("Battery node started (initially full).")

        self.on_time = 7
        self.charge_time = 3
        self.current_time = 0
        self.cycle_time = 0

        self.battery_clock()

    def battery_clock(self):
        while not rospy.is_shutdown():
            self.current_time += 1
            self.cycle_time += 1

            if self.is_full:
                if self.cycle_time >= self.on_time:
                    self.is_full = False
                    rospy.loginfo("Battery is now empty.")
                    self.cycle_time = 0
                    self.toggle_led(0, 1)
                    self.toggle_led(1, 1)
                    self.toggle_led(2, 1)
                    # self.toggle_led(3, 1)
            else:
                if self.cycle_time >= self.charge_time:
                    self.is_full = True
                    rospy.loginfo("Battery is now full.")
                    self.cycle_time = 0
                    self.toggle_led(0, 0)
                    self.toggle_led(1, 0)
                    self.toggle_led(2, 0)
        
            rospy.sleep(1)


    def toggle_led(self, led_idx = 0,led_on = 0 ):
        rospy.wait_for_service('set_led')
        try:
            client = rospy.ServiceProxy('set_led', SetLED)
            resp = client(led_idx, led_on)
            if resp.success:
                rospy.loginfo(f"Successfully set LED {led_idx} to {led_on}")
            else:
                rospy.logwarn("Service call failed or invalid LED number.")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call exception: {e}")


if __name__ == '__main__':
    Battery()
```

## Create the LED Panel Node

- Create a file named led_panel.py in the `src` folder of the `exercises` package.

```bash
cd ~/catkin_ws/src/exercises/src
touch led_panel.py
```

- Open the file and add the following code:

```python
#!/usr/bin/env python
import rospy
from exercises.srv import SetLED, SetLEDResponse

class LEDPanel:
    def __init__(self):
        rospy.init_node('led_panel')
        self.led_states = [0, 0, 0]

        self.srv = rospy.Service('set_led', SetLED, self.handle_set_led)

        rospy.Timer(rospy.Duration(1), self.print_states)

        rospy.loginfo("LED panel service ready.")
        rospy.spin()

    def handle_set_led(self, req):
        if 0 <= req.led_number < len(self.led_states):
            self.led_states[req.led_number] = req.state
            rospy.loginfo(f"LED {req.led_number} set to {req.state}")
            return SetLEDResponse(success=True)
        else:
            rospy.logwarn(f"Invalid LED number: {req.led_number}")
            return SetLEDResponse(success=False)

    def print_states(self, event):
            rospy.loginfo(f"LED states: {self.led_states}")

if __name__ == '__main__':
    LEDPanel()
```

## Make the scripts executable

```bash
cd ~/catkin_ws/src/exercises/src
chmod u+x battery.py
chmod u+x led_panel.py
```

## Build the package

```bash
cd ~/catkin_ws
catkin_make
```
