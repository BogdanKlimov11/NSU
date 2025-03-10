import numpy as np


def softmax(predictions):
    """
    Computes probabilities from scores

    Arguments:
      predictions, np array, shape is either (N) or (batch_size, N) -
        classifier output

    Returns:
      probs, np array of the same shape as predictions -
        probability for every class, 0..1
    """
    pred_temp = np.copy(predictions)
    if pred_temp.ndim == 1:
        pred_temp -= np.max(predictions)
        exp_pred = np.exp(pred_temp)
        exp_sum = np.sum(exp_pred)
        return exp_pred / exp_sum
    else:
        pred_temp = (pred_temp.T - np.max(predictions, axis=1)).T
        exp_pred = np.exp(pred_temp)
        exp_sum = np.sum(exp_pred, axis=1)
        return (exp_pred.T / exp_sum).T


def cross_entropy_loss(probs, target_index):
    """
    Computes cross-entropy loss

    Arguments:
      probs, np array, shape is either (N) or (batch_size, N) -
        probabilities for every class
      target_index: np array of int, shape is (1) or (batch_size) -
        index of the true class for given sample(s)

    Returns:
      loss: single value
    """
    if probs.ndim == 1:
        loss = -np.log(probs[target_index])
    else:
        batch_size = probs.shape[0]
        loss_n = -np.log(probs[range(batch_size), target_index])
        loss = np.sum(loss_n) / batch_size

    return loss


def l2_regularization(W, reg_strength):
    """
    Computes L2 regularization loss on weights and its gradient

    Arguments:
      W, np array - weights
      reg_strength - float value

    Returns:
      loss, single value - l2 regularization loss
      gradient, np.array same shape as W - gradient of weight by l2 loss
    """
    loss = reg_strength * np.sum(np.square(W))
    grad = 2 * reg_strength * W
    return loss, grad


def softmax_with_cross_entropy(predictions, target_index):
    """
    Computes softmax and cross-entropy loss for model predictions,
    including the gradient

    Arguments:
      predictions, np array, shape is either (N) or (batch_size, N) -
        classifier output
      target_index: np array of int, shape is (1) or (batch_size) -
        index of the true class for given sample(s)

    Returns:
      loss, single value - cross-entropy loss
      dprediction, np array same shape as predictions - gradient of predictions by loss value
    """
    probs = softmax(predictions)
    loss = cross_entropy_loss(probs, target_index)
    dprediction = probs
    if probs.ndim == 1:
        dprediction[target_index] -= 1
    else:
        batch_size = probs.shape[0]
        dprediction[range(batch_size), target_index] -= 1
        dprediction /= batch_size
    return loss, dprediction


class Param:
    """
    Trainable parameter of the model
    Captures both parameter value and the gradient
    """

    def __init__(self, value):
        self.value = value
        self.grad = np.zeros_like(value)


class ReLULayer:
    def __init__(self):
        self.X = None

    def forward(self, X):
        self.X = X
        return np.where(X < 0, 0, X)

    def backward(self, d_out):
        return np.where(self.X < 0, 0, 1) * d_out

    def params(self):
        return {}


class FullyConnectedLayer:
    def __init__(self, n_input, n_output):
        self.W = Param(0.001 * np.random.randn(n_input, n_output))
        self.B = Param(0.001 * np.random.randn(1, n_output))
        self.X = None

    def forward(self, X):
        self.X = X
        return np.dot(X, self.W.value) + self.B.value

    def backward(self, d_out):
        self.W.grad += np.dot(self.X.T, d_out)
        self.B.grad += np.dot(np.ones((1, d_out.shape[0])), d_out)
        return np.dot(d_out, self.W.value.T)

    def params(self):
        return {'W': self.W, 'B': self.B}


class ConvolutionalLayer:
    def __init__(self, in_channels, out_channels,
                 filter_size, padding):
        """
        Initializes the layer

        Arguments:
        in_channels, int - number of input channels
        out_channels, int - number of output channels
        filter_size, int - size of the conv filter
        padding, int - number of 'pixels' to pad on each side
        """

        self.X = None
        self.filter_size = filter_size
        self.in_channels = in_channels
        self.out_channels = out_channels
        self.W = Param(
            np.random.randn(filter_size, filter_size,
                            in_channels, out_channels)
        )

        self.B = Param(np.zeros(out_channels))

        self.padding = padding

    def forward(self, X):
        batch_size, height, width, channels = X.shape

        out_height = height - self.filter_size + 2 * self.padding + 1
        out_width = width - self.filter_size + 2 * self.padding + 1

        self.X = np.pad(X, ((0, 0), (self.padding, self.padding), (self.padding, self.padding), (0, 0)), 'constant',
                        constant_values=0)
        W = self.W.value.reshape(self.filter_size * self.filter_size * self.in_channels, self.out_channels)

        output = np.zeros((batch_size, out_height, out_width, self.out_channels))

        for y in range(out_height):
            for x in range(out_width):
                endX, endY = x + self.filter_size, y + self.filter_size
                segment = self.X[:, y:endY, x:endX, :].reshape(batch_size, -1)
                output[:, y, x, :] = np.dot(segment, W)

        return output + self.B.value

    def backward(self, d_out):
        # Hint: Forward pass was reduced to matrix multiply
        # You already know how to backprop through that
        # when you implemented FullyConnectedLayer
        # Just do it the same number of times and accumulate gradients

        batch_size, height, width, channels = self.X.shape
        _, out_height, out_width, out_channels = d_out.shape
        dX = np.zeros((batch_size, height, width, channels))
        W = self.W.value.reshape(self.filter_size * self.filter_size * self.in_channels, self.out_channels)
        for x in range(out_width):
            for y in range(out_height):
                segment = self.X[:, y:y + self.filter_size, x:x + self.filter_size, :]
                segment_arr = segment.reshape(batch_size, self.filter_size * self.filter_size * self.in_channels)
                d_local = d_out[:, y:y + 1, x:x + 1, :]
                dX_arr = np.dot(d_local.reshape(batch_size, -1), W.T)
                dX[:, y:y + self.filter_size, x:x + self.filter_size, :] += dX_arr.reshape(segment.shape)
                dW = np.dot(segment_arr.T, d_local.reshape(batch_size, -1))
                dB = np.dot(np.ones((1, d_local.shape[0])), d_local.reshape(batch_size, -1))
                self.W.grad += dW.reshape(self.W.value.shape)
                self.B.grad += dB.reshape(self.B.value.shape)
        return dX[:, self.padding: (height - self.padding), self.padding: (width - self.padding), :]

    def params(self):
        return {'W': self.W, 'B': self.B}


class MaxPoolingLayer:
    def __init__(self, pool_size, stride):
        """
        Initializes the max pool

        Arguments:
        pool_size, int - area to pool
        stride, int - step size between pooling windows
        """
        self.channel_indices = None
        self.batch_indices = None
        self.pool_size = pool_size
        self.stride = stride
        self.X = None

    def forward(self, X):
        self.X = X
        batch_size, height, width, channels = self.X.shape
        out_height = int((height - self.pool_size) / self.stride + 1)
        out_width = int((width - self.pool_size) / self.stride + 1)

        output = np.zeros((batch_size, out_height, out_width, channels))

        for y in range(out_height):
            for x in range(out_width):
                output[:, y, x, :] += np.amax(X[:, y * self.stride:y * self.stride + self.pool_size,
                                              x * self.stride:x * self.stride + self.pool_size, :], axis=(1, 2))
        return output

    def backward(self, d_out):
        batch_size, height, width, channels = self.X.shape
        _, out_height, out_width, out_channels = d_out.shape
        dX = np.zeros_like(self.X)
        self.batch_indices = np.arange(batch_size).repeat(channels).reshape((batch_size, channels))
        self.channel_indices = np.arange(channels).repeat(batch_size).reshape((channels, batch_size)).T

        for y in range(out_height):
            for x in range(out_width):
                slice_X = self.X[:,
                          y * self.stride:y * self.stride + self.pool_size,
                          x * self.stride:x * self.stride + self.pool_size,
                          :].reshape(batch_size, -1, channels)
                max_indices = np.unravel_index(np.argmax(slice_X, axis=1), (self.pool_size, self.pool_size))
                dX[self.batch_indices, max_indices[0] + y * self.stride, max_indices[1] + x * self.stride, self.channel_indices] += d_out[:, y, x, :]
        return dX


def params(self):
    return {}


class Flattener:
    def __init__(self):
        self.X_shape = None

    def forward(self, X):
        batch_size, height, width, channels = X.shape
        self.X_shape = X.shape
        return X.reshape(batch_size, height * width * channels)

    def backward(self, d_out):
        return d_out.reshape(self.X_shape)

    def params(self):
        # No params!
        return {}
