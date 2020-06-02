import torch
import torch.nn as nn
import torch.optim as optim
import numpy as np
from tqdm import tqdm
import matplotlib.pyplot as plt
from unet import Unet

def train_net(data_path=None, learning_rate=0.001, epochs=1, validation_percentage=0.2):
    u_net = Unet()
    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    u_net.to(device)
    optimizer = optim.Adam(u_net.parameters(), lr=learning_rate)
    loss_func = nn.MSELoss()
    dataset = np.load("training_data.npy")
    print(dataset.shape)
    x = torch.Tensor([i[0] for i in dataset], device=device).view(-1, 64, 64)
    y = torch.Tensor([i[1] for i in dataset], device=device).view(-1, 64, 64)
    val_len = int(len(dataset) * validation_percentage)
    train_x = x[:-val_len]
    train_y = y[:-val_len]
    val_x = x[-val_len:]
    val_y = y[-val_len:]

    batch_size = 1

    for epoch in range(epochs):
        for i in tqdm(range(0, len(train_x), batch_size)):
            batch_x = train_x[i:i + batch_size].view(-1, 1, 64, 64)
            batch_y = train_y[i:i + batch_size].view(-1, 1, 64, 64)
            u_net.zero_grad()
            output = u_net.forward(batch_x)
            # print(output[0][0])
            loss = loss_func(output, batch_y)
            loss.backward()
            optimizer.step()

    print(loss)
    outputs = []
    with torch.no_grad():
        for i in tqdm(range(0, len(val_x))):
            out = u_net(val_x[i].view(-1, 1, 64, 64))[0][0]
            outputs.append(out)

    torch.save(u_net, 'u_net.pt')

    plt.imshow(outputs[len(outputs)-1], cmap='gray')
    plt.show()


if __name__ == "__main__":
    train_net()