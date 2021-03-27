# Pendulum-Project
Inverted Pendulum code runs a stepper motor to control the position of a cart equipped with a pendulum. 

# Issues
* One of the key things to overcome was how to implement the encoder. One of the problems was drift of the encoder when I tried to implement the code myself. I ended up using an encoder library which solved the issue because they were using X4 encoding. This means that the position is updated on every edge of both channels. The overall accuracy of the cheap encoders is pretty incredible. 

* Another issue is the speed of the stepper, overall steppers are pretty slow and acceleration limited. Translating the code from timing pulses, to steps, to inches, to speeds is a little cumbersome in the code. Overall the cheap stepper drivers have worked just fine. 
