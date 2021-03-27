# Pendulum-Project
Inverted Pendulum code runs a stepper motor to control the position of a cart equipped with a pendulum. 

# Issues
 One of the key things to overcome was how to implement the encoder. One of the problems was drift of the encoder when I tried to implement the code myself. I ended up using an encoder library which solved the issue because they were using X4 encoding. This means that the position is updated on every edge of both channels. The overall accuracy of the cheap encoders is pretty incredible. 
