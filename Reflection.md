#**MPC Control** 

**MPC Control Project**

The goals / steps of this project are the following:
* Write a MPC controller
* Fine-tune the parameters
* Get the car to circle the loop at least once.
* And report on my understanding of MPC and the process followed 
** This here markdown readme is the written report!


#Here I will consider the [rubric points](https://review.udacity.com/#!/rubrics/896/view) individually and describe how I addressed each point in my implementation.  

---

##1. Your code should compile.

I am fairly certain my code compiles and runs without exceptions.  For some reason, as observed in my PID
 controller retrospective, the onDisconnection code had a 
seg-fault exception when it tried to close the uWS, which may be specific to the platform I am on and was commented out. 

##2. Implementation

### The Model
MPC is short for Model Predictive Control.  The main idea is to take a model, use a current state and some input
functions to make a prediction on the vehicle's path, and then use actuators to have the car follow a particular path.

In the context of our project, the telemetry data we receive from the Udacity simulator provides the current
x,y coordinates of the vehicle, the angle rotation (psi), the speed and a set of WayPoints specifying
the desired path going forward that the car shoudl follow.

During the lecture notes, we had a high-level overview of kinematic vs dynamic models.  Dynamic models attempt
to truly model the physical forces that affect a vehicle, whereas Kinematic models, the one we are writing today,
are a simplified version of dynamic models but make a nice approximation.  

### Timestep Length and Elapsed Duration (N & dt)
The timestep length and elapsed duration I chose for this project was taken from the Q&A lecture.  The main
idea behind these parameters are to choose an appropriate "look ahead" length in the future to effectively 
control the car.  With the main caveat being that adding more time will increase the amount of time it takes to 
compute things, so we need to be conservative here if we want to stay real-time.  

If I were to desire having my car move faster, it may be imperative to consider larger values, but since
I sufficiently passed the project, I will leave things as they are without breaking them!

### Polynomial Fitting and MPC Preprocessing
For this project, we used an quite a bit of math, which I would like to someday go over and derive myself 
in order to form a more basic understanding.   The polynomial fitting was the way we are matching the model's predictions
with the changes needed to the actuators to meet the desired criterion of successfully driving around the map.
By setting up the equations to include constraints on the actuators (and the other state variables we estimate for),
this basically gives us the MPC that we need. 


### Model Predictive Control with Latency
A good driving tip I have learned early on is to always slow down.  For humans, the average latency to observe-decide-react
is highly dependent on a lot of variables, but the best possible is something like 200ms.  At speeds of more than 100MPH,
this is literally faster than one can "think" and react, such that even the best drivers will have difficulty handling
a car through all possible driving scenarios.  

Since I am very patient person, and I am quite behind on this class and want to ensure passage, the most effective way
I found to deal with the latency was to decrease the vehicle speed so that it was capable of doing a full lap around the track.

I imagine that fiddling with the parameters, perhaps even using a Twiddle approach if we had a truly headless simulator 
that can be ran faster by avoiding all the unnecessary graphics processing, I can significantly increase the speed
given more time.

##3. The vehicle must successfully drive a lap around the track.

I believe you'll have to verify for yourself, but I would trust myself in the car the way it drove and it
successfully made it around the lap. And again, I am very patient, so will enjoy the view :)

