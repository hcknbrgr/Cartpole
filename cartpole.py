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

# set up neural net
l1 = 4     # Layer 1 -- set up neural net with the number of inputs
l2 = 24    # layers 2 and 3 are hidden layers
l3 = 24
l4 = 7     # 3 speeds to the left, stop motor, 3 speeds to the right

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
episodeRewards = []

if exists('tensor.tar'):
    checkpoint = torch.load('tensor.tar')
    model.load_state_dict(checkpoint['model_state_dict'])
    optimizer.load_state_dict(checkpoint['optimizer_state_dict'])
    epoch_ = checkpoint['epoch']
    loss = checkpoint['loss']
    model.train()
    print(epoch_)
    # print('parameters', model.parameters(l2))




# setup hyperparameters for RL agent
gamma = .99  # discount factor, how much we weight rewards in later states
epsilon = .99   # start with lots of exploration and decay to exploitation
min_epsilon = 0.01
max_time_steps = 250.0
solved_time = 200.0
epsilon_delta = 1.0/200.0  # linear decay over 200 iterations
print("epsilon delta: ", epsilon_delta)
epsilon = epsilon-epsilon_delta*epoch_  # if we are resuming training, then epsilon needs to resume too
print("starting epsilon: ", epsilon)

# Cart Directions -
# 0 = move towards sensor
# 1 = stationary
# 2 = move away from sensor

# The state for each step = [Cart position, cart "velocity", pole angle, pole angular velocity]

epochs = 200   # how many times we want to train
losses = []   # how well is our neural net doing for each run

# Cart Directions -
# 0-2= move towards sensor
# 3 = stationary
# 4-6 = move away from sensor


# Test the cart distance for initialization.
arduino = serial.Serial('COM7', 115200)  # All communication needs to be blocking for this to work
time.sleep(2)
currentState = []
episodeRewards = []

for epoch in range(epochs):
    epoch_ += 1
    cartCenteredCounts = 0
    poleVerticalCounts = 0
    while cartCenteredCounts != 2:  # INITIALIZE CART POSITION
        line = arduino.readline()
        if line:
            string = line.decode()
            num = int(string)
            print(num)
            if 12.0 <= num <= 20.0:  # CART IS CENTERED, MAKE SURE IT STAYS THERE A FEW SECONDS
                cartCenteredCounts += 1
                if cartCenteredCounts == 2:   # IF IT'S BEEN CENTERED LONG ENOUGH THEN THE SIGNAL THAT IT'S GOOD
                    arduino.write(struct.pack('>B', 3))
                else:
                    arduino.write(struct.pack('>B', 1))
            else:
                arduino.write(struct.pack('B', 1))  # if it's not, send that's it's not...
    #  END CART INITIALIZED POSITION on board LED should turn off
    #  INITIALIZE THE ACCELEROMETER POSITION
    while poleVerticalCounts < 3:
        line = arduino.readline()
        if line:
            string = line.decode()
            num = float(string)
            print(num)
            if 90.1 < num < 94.0:  # if it's vertical then get ready to go
                poleVerticalCounts += 1
                if poleVerticalCounts == 3:
                    arduino.write(struct.pack('>B', 1))  # cart has been vertical for a total of 3 consecutive cycles
                    print("START EPOCH")
                else:
                    arduino.write(struct.pack('>B', 0))
            else:
                arduino.write(struct.pack('>B', 0))
                poleVerticalCounts = 0
    #  CART IS COMPLETELY INITIALIZED, DO AN EPISODE!  The on-board LED should turn on.

    timeStep = 0
    line = arduino.readline().strip()  # observation state = (distance, velocity, poleAngle, angularVelocity)
    if line:
        string = line.decode().split('/')
        for a in string:
            currentState.append(float(a))
        # state_ = np.array(bucketize(currentState))
        state_ = np.array(currentState)
        currentState.clear()
        state1 = torch.from_numpy(state_).float()
        status = 1
        while status == 1:
            qvals = model(state1)  # get each of the qvals from the neural net
            qvals_ = qvals.data.numpy()  # underscore is denoting changed data
            if random.random() < epsilon:
                action_ = np.random.randint(0, 7)  # 0-2 towards wall,3 stop, 4-6 away from wall
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
            # state2_ = np.array(bucketize(currentState))
            state2_ = np.array(currentState)
            state2 = torch.from_numpy(state2_).float()
            # observation state = (distance, velocity, poleAngle, angularVelocity)
            if 5.0 < currentState[0] < 30.0 and 61.0 < currentState[2] < 121.0:
                reward = 1
            else:
                print("Failed state: ", currentState)
                reward = -1
            if timeStep > solved_time:
                reward = 10
            #print("State: ", currentState, "Bucket: ", state2_)
            print("state: ", currentState)
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
        arduino.write(struct.pack('>B', 9))  #  send a '9' to signal end of episode
        if epsilon > 0.01:
            epsilon -= epsilon_delta  # linear
            print("epsilon: ", epsilon)
        episodeRewards.append(timeStep)
        if epoch % 10 == 0:
            torch.save({
                'epoch': (epoch_+1),
                'model_state_dict': model.state_dict(),
                'optimizer_state_dict': optimizer.state_dict(),
                'loss': loss,
            }, 'tensor.tar')
            print(model.parameters(l2))

arduino.close()

pylab.plot(episodeRewards)
pylab.xlabel('Episode')
pylab.ylabel('Duration')
pylab.title('Episode Duration')
pylab.savefig("EpisodeDuration%d.pdf" % epoch_, format="pdf")

pylab.figure(figsize=(15, 10))
pylab.ylabel("Loss")
pylab.xlabel("Training Steps")
pylab.plot(losses)
pylab.savefig("dqn_loss_plot%d.pdf" % epoch_, format="pdf")

