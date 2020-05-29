import torch
import torch.nn as nn
import torch.nn.functional as F
from torchsummary import summary

class Layer(nn.Module):

    def __init__(self, input_size, output_size, batch_norm=True, kernel_size=3, padding=1, residual=True):
        super(Layer, self).__init__()
        self.input_size = input_size
        self.output_size = output_size
        self.batch_norm = batch_norm
        self.residual = residual

        self.conv1 = nn.Conv2d(input_size, output_size, kernel_size=kernel_size, padding=padding)
        self.batch1 = nn.BatchNorm2d(output_size)
        self.conv2 = nn.Conv2d(output_size, output_size, kernel_size=kernel_size, padding=padding)
        self.batch2 = nn.BatchNorm2d(output_size)
        if self.residual:
            self.conv3 = nn.Conv2d(input_size, output_size, kernel_size=3, padding=1)
            self.batch3 = nn.BatchNorm2d(output_size)

    def forward(self, input_d):
        output2 = 0

        output = self.conv1(input_d)
        if self.batch_norm:
            ioutput = self.batch1(output)
        output = nn.ReLU()(output)

        output = self.conv2(output)
        if self.batch_norm:
            output = self.batch2(output)
        output = nn.ReLU()(output)

        if self.residual:
            output2 = self.conv3(input_d)
            if self.batch_norm:
                output2 = self.batch3(output2)
            output2 = nn.ReLU()(output2)

        return nn.ReLU()(output + output2)


class Unet(nn.Module):
    def __init__(self, input_size=1, output_size=1, first_filter_size=64, kernel_size=3, padding=1):
        super().__init__()
        self.input_size = input_size
        self.output_size = output_size
        self.first_filter_size = first_filter_size
        self.kernel_size = kernel_size
        self.padding = padding

        self.conv1 = Layer(input_size, first_filter_size)

        self.conv2 = Layer(first_filter_size * 2 ** 0, first_filter_size * 2 ** 1)
        self.conv3 = Layer(first_filter_size * 2 ** 1, first_filter_size * 2 ** 2)
        self.conv4 = Layer(first_filter_size * 2 ** 2, first_filter_size * 2 ** 3)
        self.conv5 = Layer(first_filter_size * 2 ** 3, first_filter_size * 2 ** 4)

        self.conv6 = Layer(first_filter_size * 2 ** 4 + first_filter_size * 2 ** 3, first_filter_size * 2 ** 3)
        self.conv7 = Layer(first_filter_size * 2 ** 3 + first_filter_size * 2 ** 2, first_filter_size * 2 ** 2)
        self.conv8 = Layer(first_filter_size * 2 ** 2 + first_filter_size * 2 ** 1, first_filter_size * 2 ** 1)
        self.conv9 = Layer(first_filter_size * 2 ** 1 + first_filter_size * 2 ** 0, first_filter_size * 2 ** 0)
        self.conv10 = Layer(first_filter_size * 2 ** 0, output_size, residual=False)

    def forward(self, input_data):
        output_data0 = self.conv1(input_data)

        output_data1 = F.max_pool2d(output_data0, 2)
        output_data2 = self.conv2(output_data1)
        output_data3 = F.max_pool2d(output_data2, 2)
        output_data4 = self.conv3(output_data3)
        output_data5 = F.max_pool2d(output_data4, 2)
        output_data6 = self.conv4(output_data5)
        output_data7 = F.max_pool2d(output_data6, 2)
        output_data8 = self.conv5(output_data7)

        output_data = F.interpolate(output_data8, scale_factor=2)
        output_data = torch.cat([output_data, output_data6], dim=1)
        output_data = self.conv6(output_data)
        output_data = F.interpolate(output_data, scale_factor=2)
        output_data = torch.cat([output_data, output_data4], dim=1)
        output_data = self.conv7(output_data)
        output_data = F.interpolate(output_data, scale_factor=2)
        output_data = torch.cat([output_data, output_data2], dim=1)
        output_data = self.conv8(output_data)
        output_data = F.interpolate(output_data, scale_factor=2)
        output_data = torch.cat([output_data, output_data0], dim=1)
        output_data = self.conv9(output_data)

        output_data = self.conv10(output_data)

        return output_data


if __name__ == "__main__":
    u_network = Unet()
    summary(u_network, (1, 64, 64))