import tflearn
from tflearn.layers.core import input_data, dropout, fully_connected
from tflearn.layers.conv import conv_2d, max_pool_2d
from tflearn.layers.normalization import local_response_normalization
from tflearn.layers.estimator import regression
import numpy as np

# Building 'AlexNet'
network = input_data(shape=[None, 15, 40, 1])
network = conv_2d(network, 96, 11, strides=4, activation='relu')
network = max_pool_2d(network, 3, strides=2)
network = local_response_normalization(network)
network = conv_2d(network, 256, 5, activation='relu')
network = max_pool_2d(network, 3, strides=2)
network = local_response_normalization(network)
network = conv_2d(network, 384, 3, activation='relu')
network = conv_2d(network, 384, 3, activation='relu')
network = conv_2d(network, 256, 3, activation='relu')
network = max_pool_2d(network, 3, strides=2)
network = local_response_normalization(network)
network = fully_connected(network, 4096, activation='tanh')
network = dropout(network, 0.5)
network = fully_connected(network, 4096, activation='tanh')
network = dropout(network, 0.5)
network = fully_connected(network, 12, activation='softmax')
network = regression(network, optimizer='momentum', loss='categorical_crossentropy', learning_rate=0.001)

# Training
model = tflearn.DNN(network, checkpoint_path='model_alexnet', max_checkpoints=1, tensorboard_verbose=2)

model.load('alexnet.model')


def position(x):
    x = x.reshape([-1, 15, 40, 1])
    x = np.round(model.predict(x)[0])
    return x
