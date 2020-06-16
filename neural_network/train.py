import torch
import torch.nn as nn
import torch.optim as optim
import numpy as np
from tqdm import tqdm
import matplotlib.pyplot as plt
from unet import Unet, Layer
import time
import cv2

MODEL_NAME = f"model-{int(time.time())}"  # gives a dynamic model name, to just help with things getting messy over time.
learning_rate = 0.001
epochs = 4
validation_percentage = 0.1

u_net = Unet()
device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
u_net.to(device)
optimizer = optim.Adam(u_net.parameters(), lr=learning_rate)
loss_func = nn.MSELoss()

def filter_img(img00, img01):
    kernel = np.ones((4, 4), np.uint8)

    subtract = cv2.subtract((img00 + 15), img01)

    kernel2 = np.array([[-1, -1, -1], [-1, 9, -1], [-1, -1, -1]])
    img03 = cv2.filter2D(subtract, -1, kernel2)
    img03 = cv2.GaussianBlur(img03, (5, 5), 0)
    img03 = cv2.Canny(img03, 85, 255)
    img03 = cv2.morphologyEx(img03, cv2.MORPH_CLOSE, kernel, iterations=1)
    img03 = cv2.bitwise_not(img03)
    img03 = img03 & img00

    return img03


def train_net(training_path):

    dataset = np.load(training_path)

    print(dataset.shape)
    print(MODEL_NAME)

    x = torch.Tensor([i[0] for i in dataset], device=device).view(-1, 64, 64)
    y = torch.Tensor([i[1] for i in dataset], device=device).view(-1, 64, 64)
    val_len = int(len(dataset) * validation_percentage)
    train_x = x[:-val_len]
    train_y = y[:-val_len]
    val_x = x[-val_len:]
    val_y = y[-val_len:]

    batch_size = 10
    end = False
    outputs = []
    accuracy = 0

    with open("model.log", "a") as f:
        for epoch in range(epochs):
            for i in tqdm(range(0, len(train_x), batch_size)):
                batch_x = train_x[i:i + batch_size].view(-1, 1, 64, 64)
                batch_y = train_y[i:i + batch_size].view(-1, 1, 64, 64)

                u_net.zero_grad()
                output = u_net.forward(batch_x)
                loss = loss_func(output, batch_y)
                loss.backward()
                optimizer.step()
                if i % batch_size == 0:
                    f.write(
                        f"{MODEL_NAME},{round(i, 3)},{round(float(accuracy), 2)},{round(float(loss), 2)},{epoch}\n")

            with torch.no_grad():
                total = 0

                for k in range(0, len(val_x)):
                    out = u_net(val_x[k].view(-1, 1, 64, 64))
                    outputs.append(out)
                    img1 = np.uint8(out[0].view(64, 64) * 255)
                    img2 = np.uint8(y[k].view(64, 64) * 255)

                    ref = np.uint32(0)
                    dif = np.uint32(0)

                    for i in range(img1.shape[0]):
                        for j in range(img1.shape[1]):
                            ref += img2[i, j]
                            a1 = np.int16(img1[i, j])
                            a2 = np.int16(img2[i, j])
                            dif += abs(a1 - a2)
                    accuracy += round(1 - (dif / ref), 3)
                    total += 1

                accuracy = accuracy / total
                # print("Dokladnosc: ", accuracy)

                if accuracy > 0.97:
                    # print("Uzyskano wystarczającą dokładoność")
                    torch.save(u_net, 'u_net.pt')
                    end = True

                if end:
                    break
            if end:
                break

            torch.save(u_net, 'u_net.pt')

    img00 = np.uint8(val_x[0].view(64, 64) * 255)
    img01 = np.uint8(outputs[0].view(64, 64) * 255)
    img02 = np.uint8(val_y[0].view(64, 64) * 255)

    img10 = np.uint8(val_x[1].view(64, 64) * 255)
    img11 = np.uint8(outputs[1].view(64, 64) * 255)
    img12 = np.uint8(val_y[1].view(64, 64) * 255)

    img20 = np.uint8(val_x[2].view(64, 64) * 255)
    img21 = np.uint8(outputs[2].view(64, 64) * 255)
    img22 = np.uint8(val_y[2].view(64, 64) * 255)

    img30 = np.uint8(val_x[3].view(64, 64) * 255)
    img31 = np.uint8(outputs[3].view(64, 64) * 255)
    img32 = np.uint8(val_y[3].view(64, 64) * 255)

    fig, axs = plt.subplots(4, 3)
    fig.suptitle(f'Wyniki dla {dataset.shape[0]} próbek i {epochs} epok', fontsize=15)

    axs[0, 0].imshow(img00, cmap='gray')
    axs[0, 0].set_title('Wylosowane punkty')

    axs[0, 1].imshow(img01, cmap='gray')
    axs[0, 1].set_title('Odpowiedź sieci neuronowej')

    axs[0, 2].imshow(img02, cmap='gray')
    axs[0, 2].set_title('Ścieżka znaleziona dzięki RRT*')

    axs[1, 0].imshow(img10, cmap='gray')
    axs[1, 1].imshow(img11, cmap='gray')
    axs[1, 2].imshow(img12, cmap='gray')

    axs[2, 0].imshow(img20, cmap='gray')
    axs[2, 1].imshow(img21, cmap='gray')
    axs[2, 2].imshow(img22, cmap='gray')

    axs[3, 0].imshow(img30, cmap='gray')
    axs[3, 1].imshow(img31, cmap='gray')
    axs[3, 2].imshow(img32, cmap='gray')

    # plt.savefig('nn_out.png')
    plt.show()

def test_net(dateset_path, unet_path):
    dataset = np.load(dateset_path)
    u_net = torch.load(unet_path)
    u_net.to(device)


    x = torch.Tensor([i[0] for i in dataset], device=device).view(-1, 64, 64)
    y = torch.Tensor([i[1] for i in dataset], device=device).view(-1, 64, 64)
    outputs = []
    accuracy = 0
    total = 0

    for k in tqdm(range(0, len(x))):
        with torch.no_grad():
            out = u_net(x[k].view(-1, 1, 64, 64))
            outputs.append(out)
            img1 = np.uint8(out[0].view(64, 64) * 255)
            img2 = np.uint8(y[k].view(64, 64) * 255)

            ref = np.uint32(0)
            dif = np.uint32(0)

            for i in range(img1.shape[0]):
                for j in range(img1.shape[1]):
                    ref += img2[i, j]
                    a1 = np.int16(img1[i, j])
                    a2 = np.int16(img2[i, j])
                    dif += abs(a1 - a2)
            accuracy += round(1 - (dif / ref), 3)
            total += 1

    accuracy = accuracy / total
    print("Dokladnosc: ", accuracy)

    img00 = np.uint8(x[0].view(64, 64) * 255)
    img01 = np.uint8(outputs[0].view(64, 64) * 255)
    img02 = np.uint8(y[0].view(64, 64) * 255)
    img03 = filter_img(img00, img01)

    img10 = np.uint8(x[1].view(64, 64) * 255)
    img11 = np.uint8(outputs[1].view(64, 64) * 255)
    img12 = np.uint8(y[1].view(64, 64) * 255)
    img13 = filter_img(img10, img11)

    img20 = np.uint8(x[2].view(64, 64) * 255)
    img21 = np.uint8(outputs[2].view(64, 64) * 255)
    img22 = np.uint8(y[2].view(64, 64) * 255)
    img23 = filter_img(img20, img21)

    img30 = np.uint8(x[3].view(64, 64) * 255)
    img31 = np.uint8(outputs[3].view(64, 64) * 255)
    img32 = np.uint8(y[3].view(64, 64) * 255)
    img33 = filter_img(img30, img31)

    fig, axs = plt.subplots(4, 4)
    fig.suptitle(f'Wyniki testów dla {dataset.shape[0]} próbek i {epochs} epok', fontsize=15)

    axs[0, 0].imshow(img00, cmap='gray')
    axs[0, 0].set_title('Wylosowane punkty')

    axs[0, 1].imshow(img01, cmap='gray')
    axs[0, 1].set_title('Odpowiedź sieci neuronowej')

    axs[0, 2].imshow(img02, cmap='gray')
    axs[0, 2].set_title('Ścieżka znaleziona dzięki RRT*')

    axs[0, 3].imshow(img03, cmap='gray')
    axs[0, 3].set_title('Przefiltrowana ścieżka')

    axs[1, 0].imshow(img10, cmap='gray')
    axs[1, 1].imshow(img11, cmap='gray')
    axs[1, 2].imshow(img12, cmap='gray')
    axs[1, 3].imshow(img13, cmap='gray')

    axs[2, 0].imshow(img20, cmap='gray')
    axs[2, 1].imshow(img21, cmap='gray')
    axs[2, 2].imshow(img22, cmap='gray')
    axs[2, 3].imshow(img23, cmap='gray')

    axs[3, 0].imshow(img30, cmap='gray')
    axs[3, 1].imshow(img31, cmap='gray')
    axs[3, 2].imshow(img32, cmap='gray')
    axs[3, 3].imshow(img33, cmap='gray')

    # plt.savefig('nn_out.png')
    plt.show()

if __name__ == "__main__":
    train_net('training_data.npy')
    # test_net('testing_data_3.npy', 'u_net.pt')
