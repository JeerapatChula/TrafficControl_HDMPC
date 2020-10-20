import numpy as np
from keras.models import Sequential
from keras.layers import Dense
from keras.layers import Dropout
from keras.layers import LSTM
from keras.layers import SimpleRNN
from keras.layers.convolutional_recurrent import ConvLSTM2D
from keras.layers.normalization import BatchNormalization

class Model():

    @staticmethod
    def get_input_vector(num_veh, mane_veh, upper_data, old_traffic_signal, old_flow):

        num_veh = np.array(num_veh).T/200
        mane_veh = np.array(mane_veh).T/200
        upper_data = np.array(upper_data).T
        old_traffic_signal = np.array(old_traffic_signal).T/300
        old_flow = np.array(old_flow).T

        input_vector = num_veh
        input_vector = np.hstack((input_vector, mane_veh))
        input_vector = np.hstack((input_vector, upper_data))
        input_vector = np.hstack((input_vector, old_flow))
        input_vector = np.hstack((input_vector, old_traffic_signal))
        return input_vector.reshape((1, -1))

    @staticmethod
    def get_input_vector_series(num_veh, mean_veh, upper_data, old_traffic_signal, old_flow, old_vector):
        limit = 4

        num_veh = np.array(num_veh)/200
        mane_veh = np.array(mean_veh)/200
        upper_data = np.array(upper_data)
        old_traffic_signal = np.array(old_traffic_signal)/300
        old_flow = np.array(old_flow)

        input_vector = [num_veh.tolist() + mane_veh.tolist() + upper_data.tolist() + old_flow.tolist() + old_traffic_signal.tolist()]
        input_length = len(input_vector[0])


        if len(old_vector) == 0:
            old_vector = [[0] * input_length] * 3

        old_vector.append(input_vector[0])

        if len(old_vector) > limit:
            old_vector.pop(0)

            # print("///")
        # input(old_vector)
        new_vector = np.array(old_vector).reshape(limit, input_length)
        # new_vector = np.array(old_vector).reshape(1, limit, input_length)

        # input(old_vector)
        # input(new_vector)
        return new_vector, old_vector

    @staticmethod
    def model_1(input_size, output_size):
        #create model
        model = Sequential()
        #add model layers
        model.add(Dense(64, input_shape=(input_size,), activation='tanh'))
        model.add(Dense(32, activation='tanh'))
        model.add(Dense(32, activation='tanh'))
        model.add(Dense(output_size, activation='sigmoid'))

        #compile model using accuracy to measure model performance
        model.compile(optimizer='adam', loss='mean_squared_error')
        # model.summary()

    @staticmethod
    def model_2(input_size, output_size):
        #create model
        model = Sequential()
        #add model layers
        model.add(Dense(input_size*2, input_shape=(input_size,), activation='tanh'))
        model.add(Dense(input_size, activation='tanh'))
        model.add(Dense(input_size, activation='tanh'))
        model.add(Dense(output_size, activation='sigmoid'))

        #compile model using accuracy to measure model performance
        model.compile(optimizer='adam', loss='mean_squared_error')
        return model
        # model.summary()

    @staticmethod
    def model_3(input_size, output_size):
        model = Sequential()
        model.add(LSTM(input_size*2, input_shape=(4, input_size), return_sequences=True))
        model.add(LSTM(input_size*2, return_sequences=False))
        model.add(Dense(input_size, activation='tanh'))
        model.add(Dense(input_size, activation='tanh'))
        model.add(Dense(output_size, activation='sigmoid'))
        # model.add(TimeDistributed(Dense(output_size)))
        # model.add(TimeDistributed(Dense(output_size, activation='sigmoid')))
        model.compile(optimizer='adam', loss='mean_squared_error')
        # print(model.summary())
        return model

    @staticmethod
    def model_4(input_size, output_size):
        model = Sequential()
        model.add(LSTM(input_size, input_shape=(4, input_size), return_sequences=True, activation='tanh'))
        model.add(Dropout(0.1))

        model.add(LSTM(input_size, return_sequences=True, activation='tanh'))
        model.add(Dropout(0.1))

        model.add(LSTM(input_size, return_sequences=True, activation='tanh'))
        model.add(Dropout(0.1))

        model.add(LSTM(input_size, return_sequences=False, activation='tanh'))
        model.add(Dropout(0.1))

        model.add(Dense(output_size, activation='sigmoid'))
        # model.add(TimeDistributed(Dense(output_size)))
        # model.add(TimeDistributed(Dense(output_size, activation='sigmoid')))
        model.compile(optimizer='adam', loss='mean_squared_error')
        # print(model.summary())
        return model

    @staticmethod
    def model_5(input_size, output_size):
        model = Sequential()
        model.add(SimpleRNN(input_size, input_shape=(4, input_size), return_sequences=True, activation='tanh'))
        model.add(Dropout(0.1))

        model.add(SimpleRNN(input_size, return_sequences=False, activation='tanh'))
        model.add(Dropout(0.1))

        model.add(Dense(output_size, activation='sigmoid'))
        model.compile(optimizer='adam', loss='mean_squared_error')

        return model
