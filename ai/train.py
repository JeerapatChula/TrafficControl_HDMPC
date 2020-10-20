#!/usr/bin/env python

from __future__ import absolute_import
from __future__ import print_function

import os
import sys
sys.path.append(os.path.abspath(os.path.join('..')))
from model import Model
import numpy as np
from libs.etc.files import File
import random
import matplotlib.pyplot as plt
from model import Model


if __name__ == "__main__":
    scenario = "sathorn_full_junctions_morning_100"
    # intersection = "J{}".format(int(input("Intersection : ")))

    for i in range(10):
        intersection = "J{}".format(i+1)
        # print(f"Intersection: {intersection}")

        path = "data_sets/{}/{}.mat".format(scenario, intersection)
        data = File.load_mat(path)

        data_input = data["input_vector"]
        data_output = data["output_vector"]

        input_size = data_input.shape[2]
        output_size = data_output.shape[1]

        num_data = len(data_input)

        ind = np.arange(num_data)
        random.shuffle(ind)

        num_train = int(num_data * 0.8)
        num_test = num_data - num_train

        ind_train = np.arange(num_train)
        ind_test = np.arange(num_train+1, num_data)

        (X_train, y_train) = (data_input[ind_train,:], data_output[ind_train,:])
        (X_test, y_test) = (data_input[ind_test,:], data_output[ind_test,:])
        # print(X_train.shape)

        # print(X_train.shape)
        # print(y_train.shape)
        # #
        # print(X_test.shape)
        # print(y_test.shape)
        #
        # X_train = X_train.reshape(1, num_train, input_size)
        # y_train = y_train.reshape(1, num_train, output_size)
        # X_test = X_test.reshape(1, num_test, input_size)
        # y_test = y_test.reshape(1, num_test, output_size)

        # input(data_input.shape[2])

        model = Model.model_5(input_size, output_size)

        #train the model
        history = model.fit(X_train, y_train, validation_data=(X_test, y_test), epochs=200, batch_size=num_train, verbose=0)
        print("{} => Loss: {}".format(intersection, history.history['loss'][-1]))

        # "Loss"
        # plt.plot(history.history['loss'])
        # plt.plot(history.history['val_loss'])
        # plt.title('model loss')
        # plt.ylabel('loss')
        # plt.xlabel('epoch')
        # plt.legend(['train', 'validation'], loc='upper left')
        # plt.show()

        path = "output_models/{}/{}.h5".format(scenario, intersection)
        File.create_path(os.path.dirname(path))
        model.save(path)
