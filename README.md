# CoffeeMotion2ndTry

The former implementation combines the kinematics constraint and TOPP, but decouple the 6-axis robot arm to two parts, which may lead to great error.  
  
Another two ways to solve the coffee motion problem (straight line motion) states below:  
1. plan the velocity and acceleration along the line in castesian space. compute the orientation of end effector based on the acceleration and get the complete joint variable (x, y, z, theta1, theta2, theta3) via time array by applying inverse kinematics.  
2. based on the acceleration upper limit as well as the coffee cup itself, compute the range of end effector's acceleration. randomly pick one path and get the joint variable, then apply TOPP.
