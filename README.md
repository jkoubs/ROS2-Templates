# ROS2 Templates for writing Topics, Services and Actions in Python and C++

# Python

## Topics

In ROS 2, topics are a fundamental communication mechanism that enables nodes to exchange data in a **publish-subscribe** manner.

### Publisher

This publisher will continuously publish velocity commands on the **/cmd_vel** topic, allowing you to control the linear and angular velocities of your robot.

Check **publisher.py** code.

### Subscriber

This subscriber will continuously listen for LaserScan messages on the **/scan** topic and print out some relevant information when a message is received.

Check **subscriber.py** code.

### Publisher and Subscriber

The robot will move forward until it detects an obstacle within 1 meter ahead, at which point it will stop and turn right. 

Check **pub_sub.py**.

## Services

In ROS 2, services can indeed be thought of as having both **client** and **server** components.

**Server**: The node that provides the service is referred to as the server. It is responsible for implementing the functionality of the service and responding to requests from clients.

**Client**: The node that consumes the service is referred to as the client. It sends requests to the server and receives responses in return.

When a **client** wants to use a service, it sends a request to the **server**. The **server** processes the request and sends a response back to the **client**.

### Service Server

Check **service_server.py** to get an example of a **service server** node code in ROS 2 using Python. In this example, we create a simple service that takes two integers as input and returns their sum.

### Service Client

Check **service_client.py** to get an example of a **service client**. Overall, this code sets up a ROS 2 **service client** node that sends a request to a **service server** (add_two_ints) to add two integers together (in this case, 1 and 2). When the response is received from the server, it logs the sum of the two integers.

## Actions

Similarly to services, actions have both **client** and **server** components.

Actions provide a way to perform **long-running, goal-oriented tasks** with **feedback** and **result handling**. The interaction between the client and the server is more complex than with topics or services, as it involves multiple messages and states.

The **action client** is responsible for sending goals to the action server and handling the responses.
It sends a goal message to the action server and waits for feedback and result messages.
It can also cancel goals if needed.

The **action server** is responsible for receiving goals from action clients, executing the requested task, and providing feedback and results.
It accepts incoming goals and starts executing them.
It sends feedback messages to the action client to provide progress updates.
Once the task is completed, it sends a result message to the action client.

### Action Server

The **action_server.py** code sets up an **action server** node that listens for navigation goals and simulates the navigation process by sending feedback messages until the navigation is completed. Additionally, it listens for navigation goals on the navigation_goal topic and logs the received navigation goals.

### Action Client

The **action_client.py** code sets up an **action client** node that sends a navigation goal to the **move_base** topic. It also listens for feedback from the action server during goal execution.

# C++

## Topics

### Publisher

This publisher will continuously publish velocity commands on the **/cmd_vel** topic, allowing you to control the linear and angular velocities of your robot.

Check **publisher.cpp** code.

### Subscriber

This subscriber will continuously listen for LaserScan messages on the **/scan** topic and print out some relevant information when a message is received.

Check **subscriber.cpp** code.

### Publisher and Subscriber

The robot will move forward until it detects an obstacle within 1 meter ahead, at which point it will stop and turn right. 

Check **pub_sub.cpp**.

## Services

### Service Server

Check **service_server.cpp** to get an example of a **service server** node code in ROS 2 using Python. In this example, we create a simple service that takes two integers as input and returns their sum.

### Service Client

Check **service_client.cpp** to get an example of a **service client**. Overall, this code sets up a ROS 2 **service client** node that sends a request to a **service server** (add_two_ints) to add two integers together (in this case, 1 and 2). When the response is received from the server, it logs the sum of the two integers.

## Actions

### Action Server

The **action_server.cpp** code sets up an **action server** node that listens for navigation goals and simulates the navigation process by sending feedback messages until the navigation is completed. Additionally, it listens for navigation goals on the navigation_goal topic and logs the received navigation goals.

### Action Client

The **action_client.cpp** code sets up an **action client** node that sends a navigation goal to the **move_base** topic. It also listens for feedback from the action server during goal execution.