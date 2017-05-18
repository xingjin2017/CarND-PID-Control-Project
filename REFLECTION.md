## Reflection of PID tuning

The initial PID values are chosen by limiting the driving to 100 ~ 200 steps, and run the simulator with a Twiddle which updates the parameters and resets the simulator after that number of steps.

The value I initially got from this is for PID to be around 2.0, 0.0 and 10.0. Then Twiddle is used again, but with a larger number of steps to drive, say, 500 steps or 1000 steps, which covers a large portion of the track.

This works and keeps the car in the center of the road, but the steering angle is high and driving speed is slow. Later noticed that as long as KP and KD has a ratio of 1:7, it seems to work. And the lower these two numbers, the faster the car is able to drive, but with wider swings. Eventually I settled around 0.56, 0.0, and 4.12 for PID.

The throttle value being used is the default 0.3. If a lower value is used, the car will drive slower but in a much more straight line. This makes sense: the slower it is, the easier it is to drive stably and to control.
