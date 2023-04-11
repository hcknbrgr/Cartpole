import numpy as np
import torch
import random
from matplotlib import pylab as plt




# set up neural net
l1 = 4     # Layer 1 -- set up neural net with the number of inputs
l2 = 150    # layers 2 and 3 are hidden layers
l3 = 100
l4 = 3      # the answers we need

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


# setup hyperparameters for RL agent

gamma = .99  # discount factor, how much we weight rewards in later states
epsilon = 1.0   # start with lots of exploration and decay to exploitation

# Cart Directions -
# 0 = move towards sensor
# 1 = stationary
# 2 = move away from sensor

# The state for each step = [Cart position, cart "velocity", pole angle, pole angular velocity]

epochs = 100   # how many times we want to train
losses = []   # how well is our neural net doing for each run

for epoch in range(epochs):
    # TODO: INSERT INITIALIZE CART POSITION CODE

    # TODO: INITIALIZE SENSOR IN VERTICAL POSITION

    state_ = # TODO:  read in the initial state from the sensor, convert to np array
    #     state_ = game.board.render_np().reshape(1, 64) + np.random.rand(1, 64)/10.0
    state1 = torch.from_numpy(state_).float()
    status = 1
    while status == 1:
        qvals = model(state1)    # get each of the qvals from the neural net
        qvals_ = qvals.data.numpy()   # underscore is denoting changed data
        if random.random() < epsilon:
            action_ = np.random.randint(0, 3) # high is exclusive, send a random direction
        else:
            action_ = np.argmax(qvals_)
        action = action_set[action_]
        #TODO: WRITE THE ACTION TO THE SERIAL FOR ARDUINO TO READ
        state2_ = # TODO: WAIT TO READ THE SENSOR DATA TO GET THE RESULT OF THE MOVE
        state2 = torch.from_numpy(state2_).float()
        reward = # TODO: DETERMINE WHAT TO DO FOR THE REWARD. IF THE CART DIDN'T FAIL OR EXCEED TIME STEP, ADD ONE, if it fails, 0.
        with torch.no_grad():
            newQ = model(state2) #todo: convert state2 to np array?
        maxQ = torch.max(newQ)

        if reward == 1:   # if we are moving -- this is good
            Y = reward + (gamma * maxQ)   # estimate for s'
        else:
            Y = reward

        Y = torch.Tensor([Y]).detach()
        X = qvals.squeeze()[action_]
        loss = loss_fn(X, Y)
        optimizer.zero_grad()   # update NN
        loss.backward()
        losses.append(loss.item())
        optimizer.step()
        state1 = state2
        if reward != 1:
            status = 0
        # end while loop  // end of single epoch
    if epsilon > 0.01:
        epsilon -= (1.0/epochs)  # linear


plt.figure(figsize=(15,10))
plt.ylabel("Loss")
plt.xlabel("Training Steps")
plt.plot(losses)
plt.savefig("dqn_loss_plot.pdf", format="pdf")



