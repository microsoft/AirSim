import torch.nn as nn
import torch.optim as optim
import torch
import torch.nn.functional as functional
import numpy as np

class DQNetwork(nn.Module):
    def __init__(self, learningRate: float, num_actions: int, image_input_dims: tuple):
        super(DQNetwork, self).__init__()
        self.learningRate = learningRate
        self.num_actions = num_actions
        self.image_input_dims = image_input_dims

        self.maxpooling = nn.MaxPool2d((2, 2), stride=2)

        self.image_conv1 = nn.Conv2d(image_input_dims[0], 16, kernel_size=(6, 6), stride=(2, 2))
        self.image_conv2 = nn.Conv2d(16, 32, kernel_size=(3, 3), stride=(1, 1))

        self.vel_fc1 = nn.Linear(3, 16)

        conv_output_dim = self.calculate_conv_output_dims()
        self.out_fc1 = nn.Linear(conv_output_dim + 16, 16)
        self.out_fc2 = nn.Linear(16, num_actions)

        self.optimizer = optim.RMSprop(self.parameters(), lr=learningRate)
        self.loss = nn.MSELoss()
        self.device = torch.device('cuda:0')
        self.to(self.device)

    def calculate_conv_output_dims(self):
        state = torch.zeros(1, *self.image_input_dims).float()
        print("inpute state :", state.size())

        x = self.maxpooling(functional.relu(self.image_conv1(state)))
        print("layer 1", x.size())
        x = self.maxpooling(functional.relu(self.image_conv2(x)))
        print("layer 2", x.size())

        return int(np.prod(x.size()))

    def forward(self, image : torch.tensor, velocity : torch.tensor):
        image = self.maxpooling(functional.relu(self.image_conv1(image)))
        image = self.maxpooling(functional.relu(self.image_conv2(image)))
        image_flattened = image.view(image.size()[0], -1)

        velocity = functional.relu(self.vel_fc1(velocity))

        concatinated_tensor = torch.cat((image_flattened, velocity), 1)

        x = functional.relu(self.out_fc1(concatinated_tensor))
        x = self.out_fc2(x)
        return x

    def test(self):
        print("Testing network")
        image = torch.zeros(1, *self.image_input_dims).float().to(self.device)
        velocity = torch.zeros((1, 3)).float().to(self.device)
        print("Input shapes: [image]: {} [velocity]: {}".format(image.size(), velocity.size()))
        output = self.forward(image, velocity)
        print("Output: {}".format(output))

if __name__ == "__main__":
    print("test")
    model = DQNetwork(learningRate=0.001, num_actions=2, image_input_dims=(2, 64, 64))
    print("total parameters: ", sum(p.numel() for p in model.parameters()))
    print("total trainable parameters: ", sum(p.numel() for p in model.parameters() if p.requires_grad))
    print("total data points: ", (10 * 32 * 5000) / 30)
    model.test()
