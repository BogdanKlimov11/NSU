import numpy as np

from layers import (
    FullyConnectedLayer, ReLULayer,
    ConvolutionalLayer, MaxPoolingLayer, Flattener,
    softmax_with_cross_entropy
)


class ConvNet:
    """
    Implements a very simple conv net

    Input -> Conv[3x3] -> Relu -> Maxpool[4x4] ->
    Conv[3x3] -> Relu -> MaxPool[4x4] ->
    Flatten -> FC -> Softmax
    """

    def __init__(self, input_shape, n_output_classes, conv1_channels, conv2_channels):
        """
        Initializes the neural network

        Arguments:
        input_shape, tuple of 3 ints - image_width, image_height, n_channels
                                         Will be equal to (32, 32, 3)
        n_output_classes, int - number of classes to predict
        conv1_channels, int - number of filters in the 1st conv layer
        conv2_channels, int - number of filters in the 2nd conv layer
        """
        width, height, channels = input_shape
        filter_size = 3
        padding = 1
        pool_size = 4
        pool_stride = 4
        left_width = width // pool_stride // pool_stride
        left_height = height // pool_stride // pool_stride
        self.layers = [ConvolutionalLayer(channels, conv1_channels, filter_size, padding),
                       ReLULayer(),
                       MaxPoolingLayer(pool_size, pool_stride),
                       ConvolutionalLayer(conv1_channels, conv2_channels, filter_size, padding),
                       ReLULayer(),
                       MaxPoolingLayer(pool_size, pool_stride),
                       Flattener(),
                       FullyConnectedLayer(left_width * left_height * conv2_channels, n_output_classes)
                       ]

    def compute_loss_and_gradients(self, X, y):
        """
        Computes total loss and updates parameter gradients
        on a batch of training examples

        Arguments:
        X, np array (batch_size, height, width, input_features) - input data
        y, np array of int (batch_size) - classes
        """
        # Before running forward and backward pass through the model,
        # clear parameter gradients aggregated from the previous pass
        for _, v in self.params().items():
            v.grad.fill(0.0)
        forward_out = X
        for layer in self.layers:
            forward_out = layer.forward(forward_out)

        loss, d_out = softmax_with_cross_entropy(forward_out, y)
        backward_out = d_out
        for layer in reversed(self.layers):
            backward_out = layer.backward(backward_out)
        return loss

    def predict(self, X):
        forward_out = X
        for layer in self.layers:
            forward_out = layer.forward(forward_out)
        return np.argmax(forward_out, axis=1)

    def params(self):
        result = {}
        name2layer = {"Conv1": self.layers[0],
                      "Conv2": self.layers[3],
                      "Fully": self.layers[7]}

        for name, layer in name2layer.items():
            for k, v in layer.params().items():
                result['{}_{}'.format(name, k)] = v

        return result
