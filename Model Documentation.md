#How to generate path


There are several things I did to generate the path.

First, I create a finite status machine. I define a function to judge if the left or right lane is empty. In the function I define different front and rear safe distance at different velocity, just as human driver would do. After judging if there are cars on the left and right lanes, I create the available states for my car.

Second, for each states, I find the leading car in the target lane. For example, if the state is LCL meaning "Lane Change Left", I find the lane to the left of my car and find the car on that lane just in front of my car. If there is a leading car, then the speed of this leading car would be my reference speed. I would set multiple target speeds according to that speed without breaking the max acceleration or jerk. Then using these target speeds I create multiple trajectories accordingly.

Then I use several cost functions to determine which trajectory has the lowest cost. The mainly concerns are if there would be collision, if it breaks the max acceleration and jerk, if the car stays in the lane and if it is quick enough.

After this, the best trajectory is generated and pass on to the simulator and the programme goes into the next circle.

I uploaded a video of the code running here:https://youtu.be/19vk-75W4Ko