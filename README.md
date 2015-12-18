# macl
Multi-agent Collaborative Localization

Multi-agent Collaborative Localization(MACL) is used in multi-agent localization
system to optimize robot localization accuracy. The main idea
is sharing information between robots. Specifically, when robot 0 detects
robot 1, robot 0 estimates robot 1’s position and send its belief to robot 1.
Robot 1 receives the message from robot 0 and combines it with its own
belief. By fusing two position estimates, robot 1 gets better estimate of its
position.

