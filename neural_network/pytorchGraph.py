import matplotlib.pyplot as plt
from matplotlib import style

def create_acc_loss_graph(file_name):

    style.use("ggplot")
    contents = open(file_name, "r").read().split("\n")
    model_name = contents[0][0:16]

    times = []
    accuracies = []
    losses = []
    epochs = []

    for c in contents:
        if model_name in c:
            name, timestamp, acc, loss, epoch = c.split(",")

            times.append(float(timestamp))
            losses.append(float(loss))

            if int(epoch) not in epochs:
                epochs.append(int(epoch))
                accuracies.append(float(acc))

    fig1 = plt.figure()

    plt.plot(epochs, accuracies, label="acc")
    fig1.legend(loc=2)
    plt.xlabel('epochs')
    plt.ylabel('accuracy')

    fig2 = plt.figure()
    plt.plot(times, losses, label="loss")
    fig2.legend(loc=2)
    plt.xlabel('timestamp')
    plt.ylabel('loss')
    plt.show()


create_acc_loss_graph("model_6000.log")