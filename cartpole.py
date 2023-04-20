from os.path import exists
import numpy as np
import torch
import random
from matplotlib import pylab
import serial
import time
import struct  # for sending data as bytes
import matplotlib.pyplot as plt

# todo: save -- model, loss, qvals, running reward (timestep at end of epoch)

# establish bounds to create buckets for state values to establish the capability for q values in a continuous environment
state_value_bounds = [None] * 4  # [Cart position, cart "velocity", pole angle, pole angular velocity]
state_value_bounds[0] = (5, 15)  # bounds are limited to +/- 5 from center position of 10cm
state_value_bounds[1] = (-255, 255)  # negative is towards wall, positive is away from wall
state_value_bounds[2] = (60, 120)      #pole angle +/- 30 degrees from vertical
state_value_bounds[3] = (-150, 150)   # angular velocity from gyroscope
no_buckets = (1, 10, 10, 8)       # utilize example bucketization from, modify no of buckets for greater control
                                # Balancing a CartPole System with Reinforcement Learning - A Tutorial
                                # by Swagat Kumar

# set up neural net
l1 = 4     # Layer 1 -- set up neural net with the number of inputs
l2 = 150    # layers 2 and 3 are hidden layers
l3 = 100
l4 = 2

model = torch.nn.Sequential(
    torch.nn.Linear(l1, l2),
    torch.nn.ReLU(),
    torch.nn.Linear(l2, l3),
    torch.nn.ReLU(),
    torch.nn.Linear(l3, l4)
)
loss_fn = torch.nn.MSELoss()
learning_rate = 1e-3
optimizer = torch.optim.Adam(model.parameters(), lr=learning_rate)
epoch_ = 0

if exists('tensor.tar'):
    checkpoint = torch.load('tensor.tar')
    model.load_state_dict(checkpoint['model_state_dict'])
    optimizer.load_state_dict(checkpoint['optimizer_state_dict'])
    epoch_ = checkpoint['epoch']
    loss = checkpoint['loss']
    model.train()
    print (epoch_)


# setup hyperparameters for RL agent
gamma = .99  # discount factor, how much we weight rewards in later states
epsilon = .99   # start with lots of exploration and decay to exploitation
min_epsilon = 0.01
max_time_steps = 250.0
solved_time = 200.0
epsilon_delta = (epsilon - min_epsilon)/(solved_time*2)
epsilon = epsilon-epsilon_delta*epoch_  # if we are resuming training, then epsilon needs to resume too
no_streaks = 0

# Cart Directions -
# 0 = move towards sensor
# 1 = stationary
# 2 = move away from sensor

# The state for each step = [Cart position, cart "velocity", pole angle, pole angular velocity]

epochs = 50   # how many times we want to train
losses = []   # how well is our neural net doing for each run


# observation state = (distance, velocity, poleAngle, angularVelocity)
def bucketize(state_value):
    bucket_indices = []
    for i in range(len(state_value)):
        if state_value[i] <= state_value_bounds[i][0]:
            # violates lower bound
            bucket_index = 0
        elif state_value[i] >= state_value_bounds[i][1]:
            # violates upper bound
            # put in the last bucket
            bucket_index = no_buckets[i] - 1
        else:  # if within acceptable range, put it in a bucket!  According to the article, only poleAngle and poleVelocity matter
            bound_width = state_value_bounds[i][1] - state_value_bounds[i][0]
            offset = (no_buckets[i]-1) * state_value_bounds[i][0] / bound_width
            scaling = (no_buckets[i]-1) / bound_width
            bucket_index = int(round(scaling*state_value[i] -offset))
        bucket_indices.append(bucket_index)
    return tuple(bucket_indices)

# Cart Directions -
# 0 = move towards sensor
# 1 = stationary
# 2 = move away from sensor


# Test the cart distance for initialization.
arduino = serial.Serial('COM7', 115200)  # All communication needs to be blocking for this to work
time.sleep(2)
currentState = []
episodeRewards = []

for epoch in range(epochs):
    epoch_ += 1
    cartCenteredCounts = 0
    poleVerticalCounts = 0
    while cartCenteredCounts != 10:  # INITIALIZE CART POSITION
        line = arduino.readline()
        if line:
            string = line.decode()
            num = int(string)
            print(num)
            if 11.0 <= num <= 14.0:  # CART IS CENTERED, MAKE SURE IT STAYS THERE A FEW SECONDS
                cartCenteredCounts += 1
                if cartCenteredCounts == 10:   # IF IT'S BEEN CENTERED LONG ENOUGH THEN THE SIGNAL THAT IT'S GOOD
                    arduino.write(struct.pack('>B', 3))
                else:
                    arduino.write(struct.pack('>B', 1))
            else:
                arduino.write(struct.pack('B', 1))  # if it's not, send that's it's not...
    #  END CART INITIALIZED POSITION on board LED should turn off
    #  INITIALIZE THE ACCELEROMETER POSITION
    while poleVerticalCounts < 5:
        line = arduino.readline()
        if line:
            string = line.decode()
            num = float(string)
            print(num)
            if 88.0 < num < 92.0:  # if it's vertical then get ready to go
                poleVerticalCounts += 1
                if poleVerticalCounts == 5:
                    arduino.write(struct.pack('>B', 1))  # cart has been vertical for a total of 5 read cycles
                    print("START EPOCH")
                else:
                    arduino.write(struct.pack('>B', 0))
            else:
                arduino.write(struct.pack('>B', 0))
    #  CART IS COMPLETELY INITIALIZED, DO AN EPISODE!  The on-board LED should turn on.

    timeStep = 0
    line = arduino.readline().strip()  # observation state = (distance, velocity, poleAngle, angularVelocity)
    if line:
        string = line.decode().split('/')
        for a in string:
            currentState.append(float(a))
        state_ = np.array(bucketize(currentState))
        currentState.clear()
        state1 = torch.from_numpy(state_).float()
        status = 1
        while status == 1:
            qvals = model(state1)  # get each of the qvals from the neural net
            qvals_ = qvals.data.numpy()  # underscore is denoting changed data
            if random.random() < epsilon:
                action_ = np.random.randint(0, 2)  # 0 towards wall, 1 away from wall
                print("Explore: ", action_)
            else:
                action_ = np.argmax(qvals_)
                print("Exploit: ", action_)
            arduino.write(struct.pack('>B', action_))  # write the direction to send the cart in
            # wait for updated state
            line = arduino.readline().strip()  # observation state = (distance, velocity, poleAngle, angularVelocity)
            if line:
                string = line.decode().split('/')
                for a in string:
                    currentState.append(float(a))
            state2_ = np.array(bucketize(currentState))
            print("State: ", currentState, "Bucket: ", state2_)
            state2 = torch.from_numpy(state2_).float()
            # observation state = (distance, velocity, poleAngle, angularVelocity)
            if 5.0 < currentState[0] < 20.0 and 60.0 < currentState[2] < 120.0:
                reward = 1
            else:
                print("Failed state: ", currentState)
                reward = -1
            if timeStep > max_time_steps:
                reward = 10
            currentState.clear()
            with torch.no_grad():
                newQ = model(state2)
            maxQ = torch.max(newQ)

            if reward == 1:  # if we are moving -- this is good
                Y = reward + (gamma * maxQ)  # estimate for s'
            else:
                Y = reward

            Y = torch.Tensor([Y]).detach()
            X = qvals.squeeze()[action_]
            loss = loss_fn(X, Y)
            optimizer.zero_grad()  # update NN
            loss.backward()
            losses.append(loss.item())
            optimizer.step()
            state1 = state2
            timeStep += 1
            if reward != 1:
                status = 0
            # end while loop  // end of single epoch
        arduino.write(struct.pack('>B', 2))  #  send a '2' to signal end of episode
        if epsilon > 0.01:
            epsilon -= epsilon_delta  # linear
        episodeRewards.append(timeStep)
        if epoch % 10 == 0:
            torch.save({
                'epoch': (epoch+epoch_),
                'model_state_dict': model.state_dict(),
                'optimizer_state_dict': optimizer.state_dict(),
                'loss': loss
            }, 'tensor.tar')



arduino.close()

pylab.plot(episodeRewards)
pylab.xlabel('Episode')
pylab.ylabel('Duration')
pylab.title('Episode Duration')
pylab.savefig("EpisodeDuration.pdf", format="pdf")

pylab.figure(figsize=(15, 10))
pylab.ylabel("Loss")
pylab.xlabel("Training Steps")
pylab.plot(losses)
pylab.savefig("dqn_loss_plot.pdf", format="pdf")

