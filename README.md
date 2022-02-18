This package contains tools for manually invoking the dispensing or ingesting operations in RMF via simple GUI.

## Installation
``` bash
git clone https://github.com/project-covsg24/rmf_dispenser_ingestor_tools
colcon build
```

## Using the tools

### Run the dispenser
``` bash
ros2 run rmf_dispenser_ingestor_tools dispenser_gui -n test_dispenser
```

It will subscribe to `dispenser_requests` topic and will start blinking the button
if a request, which matches the name of the dispenser, was received. Clicking the button will signify that dispensing is finished and a `dispenser_results` message is sent out.

You can manually send a dispenser request message via:
``` bash
ros2 topic pub /dispenser_requests rmf_dispenser_msgs/msg/DispenserRequest "time:
  sec: 0
  nanosec: 0
request_guid: 'samples'
target_guid: 'test_dispenser'
transporter_type: ''
items: []" -1
```
