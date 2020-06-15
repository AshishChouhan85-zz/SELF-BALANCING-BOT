# SELF-BALANCING-BOT

# INTRODUCTION
This is a basic code for running a self balancing robot and is still under development. The code has 
been written in Arduino IDE.
The working of the robot is satisfactory. The algorithm to run the robot uses concepts like
**I2C Communication**, **Complementary filter** and **PID tuning**. **MPU6050** has been used to calculate the 
angle of inclination of the robot with the vertical. **L293D Motor driver** has been used to run two
DC Motors of 200rpm each. The robot only performs its basic function of balancing.

# CONCEPT BEHIND THIS PROJECT
Self balancing robot works on the principle of inverted pendulum. According to it, as the mass of upper floor
increases, moment of inertia of the upper floor increases and thus its angular acceleration decreases. In simple words
as we will increase the mass of upper floor of the robot, it will deviate less from its mean position.

# CURRENT WORKING VIDEO OF THE ROBOT
[SELF BALACING BOT VIDEO]{https://drive.google.com/file/d/1_SxAagJUW3OofHMXuxUm8yrXQoLHnXQo/view?usp=sharing)

