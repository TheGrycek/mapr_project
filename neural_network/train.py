import torch
import torch.nn as nn
import torch.optim as optim
import numpy as np
from tqdm import tqdm
import matplotlib.pyplot as plt
from unet import Unet


def train_net(data_path=None, learning_rate=0.001, epochs=5, validation_percentage=0.1):
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

    batch_size = 3

    for epoch in range(epochs):
        for i in tqdm(range(0, len(train_x), batch_size)):
            batch_x = train_x[i:i + batch_size].view(-1, 1, 64, 64)
            batch_y = train_y[i:i + batch_size].view(-1, 1, 64, 64)
            u_net.zero_grad()
            output = u_net.forward(batch_x)
            loss = loss_func(output, batch_y)
            loss.backward()
            optimizer.step()

    print(loss)

    outputs = []
    correct = 0
    total = 0

    with torch.no_grad():
        for i in tqdm(range(0, len(val_x))):
            # wynik sieci
            out = u_net(val_x[i].view(-1,1, 64, 64))
            outputs.append(out)
            # przekonwertowanie do uint8 wyniku sieci i zdjecia referencyjnego
            img1 = np.uint8(out[0].view(64, 64) * 255)
            img2 = np.uint8(val_y[0].view(64, 64) * 255)

            # zmienne do obliczenia roznicy pomiedzy wynikiem sieci a zdjeciem referencyjnym
            ref = np.uint32(0)
            dif = np.uint32(0)

            for i in range(img1 .shape[0]):
                for j in range(img1 .shape[1]):
                    # suma wartosci w zdjeciu ref
                    ref += img2[i, j]
                    # roznica wartosci piksela miedzy wartoscia ref a wynikiem sieci
                    a1 = np.int16(img1[i, j])  # wartosc piksela musi byc zrzutowana na int, inaczej blad przy liczeniu dif
                    a2 = np.int16(img2[i, j])
                    #print("a1: ", a1, "a2: ", a2, "roznica: ", abs(a1-a2))
                    dif += abs(a1 - a2)

            valid = round(dif / ref , 3) # wartosci wzgledna roznicy
            print("Wynik walidacji: ",valid)

            if valid < 0.3: # jesli wartosc wzgledna roznicy mniejsza niz 10% to uznajemy ze OK
                correct += 1

            total +=1

    val_acc = round(correct / total, 3)

    print("Accuracy: ", val_acc)

    torch.save(u_net, 'u_net.pt')

    img00 = np.uint8(val_x[0].view(64, 64) * 255)
    img01 = np.uint8(outputs[0].view(64, 64) * 255)
    img02 = np.uint8(val_y[0].view(64, 64) * 255)

    img10 = np.uint8(val_x[1].view(64, 64)*255)
    img11 = np.uint8(outputs[1].view(64, 64)*255)
    img12 = np.uint8(val_y[1].view(64, 64)*255)

    plt.subplot(231)
    plt.imshow( img00, cmap='gray')

    plt.subplot(232)
    plt.imshow(img01, cmap='gray')

    plt.subplot(233)
    # plt.imshow(dataset[0][1], cmap='gray')
    plt.imshow(img02, cmap='gray')

    plt.subplot(234)
    plt.imshow(img10, cmap='gray')

    plt.subplot(235)
    plt.imshow(img11, cmap='gray')

    plt.subplot(236)
    plt.imshow(img12, cmap='gray')
    plt.show()


if __name__ == "__main__":
    train_net()